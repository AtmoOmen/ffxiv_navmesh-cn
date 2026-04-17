using vnavmesh.Bootstrap;
using vnavmesh.Shared.Utilities;

namespace vnavmesh.Navigation.Mesh.Runtime;

public sealed partial class NavmeshManager
{
    private void QueueCacheWrite(string cacheKey, FileInfo cache, Navmesh navmesh)
    {
        if (_cacheWriteTasks.TryGetValue(cacheKey, out var existing) && !existing.IsCompleted)
        {
            Log($"后台缓存写入已在进行，跳过重复调度: {cacheKey}");
            return;
        }

        var writeTask = Task.Run(() => WriteCache(cacheKey, cache, navmesh));
        _cacheWriteTasks[cacheKey] = writeTask;
        _cacheWriteTasksByNavmesh[navmesh] = writeTask;
        _ = writeTask.ContinueWith
        (
            t =>
            {
                _cacheWriteTasks.TryRemove(cacheKey, out _);
                _cacheWriteTasksByNavmesh.TryRemove(navmesh, out _);
                LogTaskError(t);
            },
            CancellationToken.None,
            TaskContinuationOptions.ExecuteSynchronously,
            TaskScheduler.Default
        );
    }

    private void WriteCache(string cacheKey, FileInfo cache, Navmesh navmesh)
    {
        var timer    = StopWatchTimer.Create();
        var tempPath = $"{cache.FullName}.{Environment.ProcessId}.{Environment.CurrentManagedThreadId}.tmp";

        try
        {
            cache.Directory?.Create();
            Service.Log.Debug($"[vnavmesh] 后台写入缓存: {cache.FullName}");
            var                    serializeTimer = StopWatchTimer.Create();
            Navmesh.CacheTelemetry telemetry;

            using (var stream = new FileStream(tempPath, FileMode.Create, FileAccess.Write, FileShare.None, 1 << 22, FileOptions.SequentialScan))
            using (var writer = new BinaryWriter(stream))
            {
                telemetry = navmesh.Serialize(writer);
                writer.Flush();
                stream.Flush();
            }

            var serializeDuration = serializeTimer.Value();
            var sizeBytes         = new FileInfo(tempPath).Length;

            var replaceTimer = StopWatchTimer.Create();
            File.Move(tempPath, cache.FullName, true);
            var replaceDuration = replaceTimer.Value();
            LogCacheSegment("写入", telemetry.Mesh);
            LogCacheSegment("写入", telemetry.Volume);
            Log
            (
                $"后台缓存写入完成 '{cacheKey}'，总耗时 {timer.Value().TotalMilliseconds:f1} ms，序列化 {serializeDuration.TotalMilliseconds:f1} ms，替换 {replaceDuration.TotalMilliseconds:f1} ms，大小 {sizeBytes / 1024.0 / 1024.0:f2} MiB"
            );
        }
        catch (Exception ex)
        {
            Log($"后台缓存写入失败 '{cacheKey}': {ex}");
        }
        finally
        {
            try
            {
                if (File.Exists(tempPath))
                    File.Delete(tempPath);
            }
            catch
            {
            }
        }
    }
}
