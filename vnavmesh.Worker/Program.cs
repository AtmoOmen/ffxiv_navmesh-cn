using System.Diagnostics;
using System.IO.Pipes;
using System.Buffers.Binary;
using System.Text;
using System.Text.Json;
using System.Threading;
using vnavmesh.Common.Ipc;
using vnavmesh.Common.Navigation.Mesh.Build;
using vnavmesh.Common.Navigation.Scene;

var pipeName = ReadOption(args, "--pipe");
var manifestPath = ReadOption(args, "--manifest");
var manifestJsonOptions = new JsonSerializerOptions
{
    IncludeFields = true
};
if (string.IsNullOrWhiteSpace(pipeName) || string.IsNullOrWhiteSpace(manifestPath))
{
    Console.Error.WriteLine("Usage: vnavmesh.Worker --pipe <name> --manifest <path>");
    return 2;
}

await using var pipe = new NamedPipeClientStream(".", pipeName, PipeDirection.Out, PipeOptions.Asynchronous);
await pipe.ConnectAsync(30000);
var writeGate         = new SemaphoreSlim(1, 1);
var progressDone      = 0;
var progressPermille  = 0;
var progressSentValue  = -1;
var reporterTask      = Task.CompletedTask;

var timer = Stopwatch.StartNew();
try
{
    var manifestJson = await File.ReadAllTextAsync(manifestPath, Encoding.UTF8);
    var manifest = JsonSerializer.Deserialize<NavmeshBuildManifest>(manifestJson, manifestJsonOptions) ??
                   throw new InvalidOperationException("构建清单为空");
    manifest.Settings.BuildMaxCores = 0;

    BuildScene scene;
    await using (var sceneStream = new FileStream(manifest.ScenePath, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 22, FileOptions.SequentialScan))
    using (var sceneReader = new BinaryReader(sceneStream, Encoding.UTF8, true))
        scene = BuildScene.Read(sceneReader);

    var builder = new NavmeshBuilder(scene, manifest.Settings);
    var totalProgressWeight = Math.Max(builder.TotalEstimatedTileWeight, 1);
    var finishedWeight      = 0L;
    reporterTask = Task.Run(async () =>
    {
        while (Volatile.Read(ref progressDone) == 0)
        {
            await ReportProgressIfNeeded();
            await Task.Delay(100);
        }

        await ReportProgressIfNeeded();

        async Task ReportProgressIfNeeded()
        {
            var current = Volatile.Read(ref progressPermille);
            if (current == progressSentValue)
                return;

            progressSentValue = current;
            await WriteMessageAsync(new NavmeshBuildMessage("progress", new(current / 1000d), null));
        }
    });

    WriteProgress(0);
    builder.Build
    (
        weight =>
        {
            var progress = Math.Min(0.99, Interlocked.Add(ref finishedWeight, weight) / (double)totalProgressWeight * 0.99);
            WriteProgress(progress);
        }
    );

    await StopProgressReporter();

    if (builder.Navmesh.Volume != null)
        builder.Navmesh.Volume.CompactRetainedState();

    Directory.CreateDirectory(Path.GetDirectoryName(manifest.RawNavmeshPath) ?? ".");
    await using (var rawStream = new FileStream(manifest.RawNavmeshPath, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 22, FileOptions.SequentialScan))
    using (var rawWriter = new BinaryWriter(rawStream, Encoding.UTF8, true))
    {
        builder.Navmesh.Serialize(rawWriter);
        rawWriter.Flush();
        await rawStream.FlushAsync();
    }

    await WriteMessageAsync(new NavmeshBuildMessage("final", null, new(true, manifest.RawNavmeshPath, null, timer.Elapsed.TotalMilliseconds)));
    return 0;
}
catch (Exception ex)
{
    await StopProgressReporter();
    await WriteMessageAsync(new NavmeshBuildMessage("final", null, new(false, null, ex.ToString(), timer.Elapsed.TotalMilliseconds)));
    return 1;
}

void WriteProgress(double progress) =>
    Interlocked.Exchange(ref progressPermille, (int)Math.Clamp(Math.Round(progress * 1000d), 0, 999));

async Task WriteMessageAsync(NavmeshBuildMessage message)
{
    await writeGate.WaitAsync();
    try
    {
        var payload = JsonSerializer.SerializeToUtf8Bytes(message, manifestJsonOptions);
        var length = new byte[sizeof(int)];
        BinaryPrimitives.WriteInt32LittleEndian(length, payload.Length);
        await pipe.WriteAsync(length);
        await pipe.WriteAsync(payload);
        await pipe.FlushAsync();
    }
    finally
    {
        writeGate.Release();
    }
}

async Task StopProgressReporter()
{
    Volatile.Write(ref progressDone, 1);

    try
    {
        await reporterTask;
    }
    catch (Exception ex)
    {
        Console.Error.WriteLine(ex);
    }
}

static string? ReadOption(string[] args, string name)
{
    for (var i = 0; i < args.Length - 1; ++i)
        if (string.Equals(args[i], name, StringComparison.OrdinalIgnoreCase))
            return args[i + 1];
    return null;
}
