namespace vnavmesh.Common.Build;

public sealed record NavmeshBuildManifest
(
    string               ScenePath,
    string               RawNavmeshPath,
    string               CacheKey,
    NavmeshBuildSettings Settings
);

public sealed record NavmeshBuildResponse
(
    bool    Success,
    string? RawNavmeshPath,
    string? Error,
    double  DurationMs
);

public sealed record NavmeshBuildProgress
(
    double Progress
);

public sealed record NavmeshBuildMessage
(
    string?               Type,
    NavmeshBuildProgress? Progress,
    NavmeshBuildResponse? Response
);
