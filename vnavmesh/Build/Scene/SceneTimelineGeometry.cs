using FFXIVClientStructs.FFXIV.Client.LayoutEngine;
using FFXIVClientStructs.FFXIV.Client.LayoutEngine.Group;

namespace vnavmesh.Build.Scene;

public static unsafe class SceneTimelineGeometry
{
    public static void Collect
    (
        SharedGroupLayoutInstance* group,
        HashSet<nint>              ignored
    )
    {
        HashSet<uint>          passable    = [];
        Dictionary<uint, nint> staticPoses = [];

        foreach (var timeline in group->TimeLineContainer.Instances)
        {
            var data = timeline.Value->DataPtr;
            if (data == null)
                continue;

            Dictionary<ushort, uint> actorInstances = [];

            foreach (ref readonly var instance in data->Instances)
            {
                fixed (FileSceneTimelineInstance* instanceData = &instance)
                    actorInstances.Add(*(ushort*)instanceData, (uint)instance.SubId);
            }

            var                      description = (FileSceneTimelineDescription*)data->Tmlb;
            Dictionary<ushort, nint> nodes       = [];
            var                      node        = &description->First;

            for (var index = 0; index < description->NumElements; ++index)
            {
                nodes.Add(*(ushort*)&node->Data, (nint)node);
                node = (FileSceneTimelineNode*)((byte*)node + node->Size);
            }

            foreach (var address in nodes.Values)
            {
                var actor = (FileSceneTimelineNode*)address;
                if (actor->Tag != 0x43414D54)
                    continue;

                var actorData = (byte*)&actor->Data;
                if (!actorInstances.TryGetValue(*(ushort*)(actorData + 2), out var subId))
                    continue;

                var trackIds = new ReadOnlySpan<ushort>(actorData + *(int*)(actorData + 12), *(int*)(actorData + 16));

                foreach (var trackId in trackIds)
                {
                    var track     = (FileSceneTimelineNode*)nodes[trackId];
                    var trackData = (byte*)&track->Data;
                    var clipIds   = new ReadOnlySpan<ushort>(trackData + *(int*)(trackData + 4), *(int*)(trackData + 8));

                    foreach (var clipId in clipIds)
                    {
                        var clip = (FileSceneTimelineNode*)nodes[clipId];

                        switch (clip->Tag)
                        {
                            case 0x35353043 when *((byte*)&clip->Data + 12) == 0:
                            case 0x33313043:
                                passable.Add(subId);
                                break;
                            case 0x38313043:
                                var pose = (float*)((byte*)&clip->Data + 12);

                                if (staticPoses.TryGetValue(subId, out var previousPose))
                                {
                                    if (!new ReadOnlySpan<float>(pose, 9).SequenceEqual(new ReadOnlySpan<float>((void*)previousPose, 9)))
                                        passable.Add(subId);
                                }
                                else
                                {
                                    staticPoses.Add(subId, (nint)pose);
                                }

                                break;
                        }
                    }
                }
            }
        }

        foreach (var child in group->Instances.Instances)
        {
            var instance  = child.Value->Instance;
            var subId     = (byte)(instance->SubId >> (8 * (4 - instance->NestingLevel)));
            var isIgnored = ignored.Contains((nint)group) || passable.Contains(subId);
            if (isIgnored)
                ignored.Add((nint)instance);

            if (instance->Id.Type == InstanceType.SharedGroup)
                Collect((SharedGroupLayoutInstance*)instance, ignored);
        }
    }
}
