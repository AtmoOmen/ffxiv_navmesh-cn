using System.Numerics;
using System.Text.Json;
using Seeds = System.Collections.Generic.Dictionary<uint, System.Collections.Generic.List<vnavmesh.Build.Ground.JsonVec>>;

namespace vnavmesh.Build.Ground;

public class FloodFill
{
    public const string REMOTE_SOURCE = "https://raw.githubusercontent.com/awgil/ffxiv_navmesh/refs/heads/seeds/seeds.json";

    private static readonly JsonSerializerOptions DeOpts = new() { ReadCommentHandling = JsonCommentHandling.Skip };

    private static readonly JsonSerializerOptions SerOpts = new()
    {
        WriteIndented = true,
        IndentSize    = 2
    };

    private static readonly HttpClient Client = new();

    private static FloodFill? Instance;

    public Action<uint, Vector3> PointAdded = delegate { };

    public static readonly string LocalSource;

    private readonly Seeds seedsRemote;
    private readonly Seeds seedsLocal;

    private FloodFill
    (
        Seeds remote,
        Seeds local
    )
    {
        seedsRemote = remote;
        seedsLocal  = local;
    }

    static FloodFill()
    {
        var dir = new DirectoryInfo(Service.PluginInterface.GetPluginConfigDirectory());
        if (!dir.Exists)
            dir.Create();

        LocalSource = Path.Join(Service.PluginInterface.GetPluginConfigDirectory(), "seeds-local.json");
    }

    internal static void Clear() => Instance = null;

    public static FloodFill? Get() => Instance;

    public static async Task<FloodFill> GetAsync()
    {
        Instance ??= await Init();
        return Instance;
    }

    public void AddPoint
    (
        uint    zone,
        Vector3 point
    )
    {
        seedsLocal.TryAdd(zone, []);
        seedsLocal[zone].Add(point);
        PointAdded.Invoke(zone, point);
    }

    public async Task Serialize()
    {
        var       finfo = new FileInfo(LocalSource);
        using var st    = finfo.Create();
        await JsonSerializer.SerializeAsync(st, new SortedDictionary<uint, List<JsonVec>>(seedsLocal), SerOpts);
    }

    private static async Task<FloodFill> Init()
    {
        Seeds remote = [];
        Seeds local  = [];

        try
        {

            using var resp = await Client.GetAsync(REMOTE_SOURCE);
            resp.EnsureSuccessStatusCode();
            var body = await resp.Content.ReadAsStreamAsync();
            remote = await FromStream(body);
        }
        catch (HttpRequestException ex)
        {
            Service.Log.Warning(ex, "Unable to fetch seeds from Github, quality will be lacking");
        }

        try
        {
            var fileInfo = new FileInfo(LocalSource);

            await using var st = fileInfo.OpenRead();
            local = await FromStream(st);
        }
        catch (FileNotFoundException ex)
        {
            Service.Log.Info(ex, "Local seeds file not found");
        }

        foreach (var (zone, seedsRemote) in remote)
        {
            if (local.TryGetValue(zone, out var seedsLocal))
            {
                // cleanup local seeds when remote is updated
                seedsLocal.RemoveAll(l => seedsRemote.Any(r => Vector3.DistanceSquared(r, l) < 10));
            }
        }

        return new(remote, local);
    }

    private static async Task<Seeds> FromStream
    (
        Stream s
    )
    {
        try
        {
            return await JsonSerializer.DeserializeAsync<Seeds>(s, DeOpts) ?? [];
        }
        catch (JsonException ex)
        {
            Service.Log.Warning(ex, "Unable to load flood points");
            return [];
        }
    }

    public bool TryLookup
    (
        uint                     territoryType,
        out IEnumerable<Vector3> points
    )
    {
        var collA = seedsRemote.TryGetValue(territoryType, out var r) ?
                        r :
                        [];
        var collB = seedsLocal.TryGetValue(territoryType, out var l) ?
                        l :
                        [];

        points = [.. collA, .. collB];
        return collA.Count > 0 || collB.Count > 0;
    }
}
