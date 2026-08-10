using System.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using DotRecast.Core.Numerics;
using DotRecast.Detour;

namespace vnavmesh.Query.Ground;

using static DtDetour;

internal static class BidirectionalGroundPathSearch
{
    private const int   MAX_CORRIDOR_POLYS                  = 4096;
    private const int   MIN_GEOMETRY_REFINEMENT_CORRIDOR    = 32;
    private const int   MAX_GEOMETRY_REFINEMENT_POINTS      = 4097;
    private const float GEOMETRY_REFINEMENT_MARGIN          = 0.25f;
    private const float GEOMETRY_REFINEMENT_CORNER_PENALTY  = 0.20f;
    private const float GEOMETRY_REFINEMENT_TURN_PENALTY    = 2.00f;

    private static readonly ConditionalWeakTable<DtNavMesh, Dictionary<long, List<long>>> s_reverseAdjacency = new();

    internal static DtStatus FindPath
    (
        DtNavMeshQuery meshQuery,
        long           startRef,
        long           endRef,
        RcVec3f        startPos,
        RcVec3f        endPos,
        IDtQueryFilter filter,
        float          heuristicScale,
        List<long>     corridor
    )
    {
        corridor.Clear();

        if (startRef == endRef)
        {
            corridor.Add(startRef);
            return DtStatus.DT_SUCCESS;
        }

        var mesh = meshQuery.GetAttachedNavMesh();
        if (!mesh.IsValidPolyRef(startRef) || !mesh.IsValidPolyRef(endRef))
            return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;

        var reverseAdjacency = s_reverseAdjacency.GetValue(mesh, static m => BuildReverseAdjacency(m));
        var forward          = new SearchState();
        var backward         = new SearchState();

        var forwardStartIndex = GetOrCreateNode(forward, startRef);
        var forwardStart      = forward.Nodes[forwardStartIndex];
        forwardStart.Pos = startPos;
        Push(forward, forwardStart, heuristicScale * RcVec3f.Distance(startPos, endPos));

        var backwardEndIndex = GetOrCreateNode(backward, endRef);
        var backwardEnd      = backward.Nodes[backwardEndIndex];
        backwardEnd.Pos = endPos;
        Push(backward, backwardEnd, heuristicScale * RcVec3f.Distance(endPos, startPos));

        var meeting              = new MeetingResult();
        var bestPartialForward   = 0;
        var bestPartialScore     = RcVec3f.Distance(startPos, endPos);
        var sourceTargetEstimate = heuristicScale * RcVec3f.Distance(startPos, endPos);

        while (true)
        {
            var hasForward  = TryPeekValid(forward,  out var forwardTop);
            var hasBackward = TryPeekValid(backward, out var backwardTop);

            if (!hasForward && !hasBackward)
                break;

            if (hasForward && hasBackward)
            {
                if (forwardTop.F + backwardTop.F >= meeting.BestPath + sourceTargetEstimate)
                    break;

                if (forwardTop.F < backwardTop.F)
                    ExpandForwardSide();
                else
                    ExpandBackwardSide();
            }
            else if (hasForward)
                ExpandForwardSide();
            else
                ExpandBackwardSide();
        }

        var status = DtStatus.DT_SUCCESS;

        if (meeting.ForwardIndex >= 0 && meeting.BackwardIndex >= 0)
            BuildCorridor(forward, backward, meeting.ForwardIndex, meeting.BackwardIndex, corridor);
        else
        {
            BuildPartialCorridor(forward, bestPartialForward, corridor);
            status |= DtStatus.DT_PARTIAL_RESULT;
        }

        status = RefineCorridorGeometry
        (
            meshQuery,
            startPos,
            endPos,
            filter,
            heuristicScale,
            corridor,
            status
        );

        if (corridor.Count > MAX_CORRIDOR_POLYS)
        {
            corridor.RemoveRange(MAX_CORRIDOR_POLYS, corridor.Count - MAX_CORRIDOR_POLYS);
            status |= DtStatus.DT_BUFFER_TOO_SMALL;
        }

        return status;

        void ExpandForwardSide()
        {
            var entry = PopValid(forward);
            var node  = forward.Nodes[entry.NodeIndex];
            node.IsOpen   = false;
            node.IsClosed = true;

            if (backward.IndexByRef.TryGetValue(node.Ref, out var backwardIndex))
            {
                var backwardNode = backward.Nodes[backwardIndex];
                if (backwardNode.IsClosed)
                    TryRecordMeeting
                    (
                        mesh,
                        filter,
                        forward,
                        entry.NodeIndex,
                        backward,
                        backwardIndex,
                        endRef,
                        meeting
                    );
            }

            var partialScore = RcVec3f.Distance(node.Pos, endPos);

            if (partialScore < bestPartialScore)
            {
                bestPartialScore   = partialScore;
                bestPartialForward = entry.NodeIndex;
            }

            ExpandForward
            (
                meshQuery,
                mesh,
                filter,
                heuristicScale,
                forward,
                backward,
                entry.NodeIndex,
                endRef,
                endPos,
                meeting
            );
        }

        void ExpandBackwardSide()
        {
            var entry = PopValid(backward);
            var node  = backward.Nodes[entry.NodeIndex];
            node.IsOpen   = false;
            node.IsClosed = true;

            if (forward.IndexByRef.TryGetValue(node.Ref, out var forwardIndex))
            {
                var forwardNode = forward.Nodes[forwardIndex];
                if (forwardNode.IsClosed)
                    TryRecordMeeting
                    (
                        mesh,
                        filter,
                        forward,
                        forwardIndex,
                        backward,
                        entry.NodeIndex,
                        endRef,
                        meeting
                    );
            }

            ExpandBackward
            (
                meshQuery,
                mesh,
                filter,
                heuristicScale,
                forward,
                backward,
                reverseAdjacency,
                entry.NodeIndex,
                endRef,
                startPos,
                endPos,
                meeting
            );
        }
    }

    private static DtStatus RefineCorridorGeometry
    (
        DtNavMeshQuery meshQuery,
        RcVec3f        startPos,
        RcVec3f        endPos,
        IDtQueryFilter filter,
        float          heuristicScale,
        List<long>     corridor,
        DtStatus       currentStatus
    )
    {
        if (currentStatus.Failed() || currentStatus.IsPartial() || corridor.Count < MIN_GEOMETRY_REFINEMENT_CORRIDOR)
            return currentStatus;

        var candidateBuffer = ArrayPool<long>.Shared.Rent(MAX_CORRIDOR_POLYS);
        var straightPath    = ArrayPool<DtStraightPath>.Shared.Rent(MAX_GEOMETRY_REFINEMENT_POINTS);

        try
        {
            var candidateStatus = meshQuery.FindPath
            (
                corridor[0],
                corridor[^1],
                startPos,
                endPos,
                filter,
                candidateBuffer.AsSpan(0, MAX_CORRIDOR_POLYS),
                out var candidateCount,
                MAX_CORRIDOR_POLYS,
                heuristicScale == 0f ? DtFindPathOption.ZeroScale : DtFindPathOption.NoOption
            );
            if (candidateStatus.Failed()                          ||
                candidateStatus.IsPartial()                       ||
                candidateStatus.Has(DtStatus.DT_BUFFER_TOO_SMALL) ||
                candidateCount == 0)
                return currentStatus;

            var straightPathBuffer = straightPath.AsSpan(0, MAX_GEOMETRY_REFINEMENT_POINTS);
            var currentQuality = MeasurePathQuality
            (
                meshQuery,
                startPos,
                endPos,
                CollectionsMarshal.AsSpan(corridor),
                straightPathBuffer
            );
            var candidateQuality = MeasurePathQuality
            (
                meshQuery,
                startPos,
                endPos,
                candidateBuffer.AsSpan(0, candidateCount),
                straightPathBuffer
            );
            if (!candidateQuality.IsValid ||
                (currentQuality.IsValid && candidateQuality.Score + GEOMETRY_REFINEMENT_MARGIN >= currentQuality.Score))
                return currentStatus;

            corridor.Clear();
            for (var i = 0; i < candidateCount; ++i)
                corridor.Add(candidateBuffer[i]);

            return candidateStatus;
        }
        finally
        {
            ArrayPool<long>.Shared.Return(candidateBuffer);
            ArrayPool<DtStraightPath>.Shared.Return(straightPath);
        }
    }

    private static PathQuality MeasurePathQuality
    (
        DtNavMeshQuery       meshQuery,
        RcVec3f              startPos,
        RcVec3f              endPos,
        Span<long>           corridor,
        Span<DtStraightPath> straightPath
    )
    {
        var status = meshQuery.FindStraightPath
        (
            startPos,
            endPos,
            corridor,
            corridor.Length,
            straightPath,
            out var count,
            straightPath.Length,
            0
        );
        if (status.Failed() || status.Has(DtStatus.DT_BUFFER_TOO_SMALL) || count == 0)
            return default;

        var length     = 0f;
        var turnAmount = 0f;
        for (var i = 1; i < count; ++i)
        {
            var previous = straightPath[i - 1].pos;
            var current  = straightPath[i].pos;
            var dx       = current.X - previous.X;
            var dz       = current.Z - previous.Z;
            length += MathF.Sqrt((dx * dx) + (dz * dz));
        }

        for (var i = 1; i < count - 1; ++i)
        {
            var previous       = straightPath[i - 1].pos;
            var current        = straightPath[i].pos;
            var next           = straightPath[i + 1].pos;
            var incomingX      = current.X - previous.X;
            var incomingZ      = current.Z - previous.Z;
            var outgoingX      = next.X - current.X;
            var outgoingZ      = next.Z - current.Z;
            var incomingLength = MathF.Sqrt((incomingX * incomingX) + (incomingZ * incomingZ));
            var outgoingLength = MathF.Sqrt((outgoingX * outgoingX) + (outgoingZ * outgoingZ));
            if (incomingLength <= 1e-4f || outgoingLength <= 1e-4f)
                continue;

            var directionDot = ((incomingX * outgoingX) + (incomingZ * outgoingZ)) / (incomingLength * outgoingLength);
            turnAmount += 1f - Math.Clamp(directionDot, -1f, 1f);
        }

        return new(length, count, turnAmount, true);
    }

    private static Dictionary<long, List<long>> BuildReverseAdjacency
    (
        DtNavMesh mesh
    )
    {
        var reverse = new Dictionary<long, List<long>>();

        for (var tileIndex = 0; tileIndex < mesh.GetMaxTiles(); ++tileIndex)
        {
            var tile = mesh.GetTile(tileIndex);
            if (tile?.data?.header == null || tile.data.offMeshCons == null || tile.links == null)
                continue;

            var baseRef = mesh.GetPolyRefBase(tile);

            for (var connectionIndex = 0; connectionIndex < tile.data.header.offMeshConCount; ++connectionIndex)
            {
                var connection = tile.data.offMeshCons[connectionIndex];
                if (connection == null || connection.poly < 0 || connection.poly >= tile.data.header.polyCount)
                    continue;

                var poly = tile.data.polys[connection.poly];
                if (poly.GetPolyType() != DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
                    continue;

                var connectionRef      = baseRef | (long)(uint)connection.poly;
                var endIsBidirectional = connection.side == 0xff || (connection.flags & DT_OFFMESH_CON_BIDIR) != 0;

                for (var linkIndex = poly.firstLink; linkIndex != DT_NULL_LINK; linkIndex = tile.links[linkIndex].next)
                {
                    var link = tile.links[linkIndex];
                    if (link.refs == 0)
                        continue;

                    AddReverseEntry(reverse, link.refs, connectionRef);

                    if (link.edge == 0 || (link.edge == 1 && endIsBidirectional))
                        AddReverseEntry(reverse, connectionRef, link.refs);
                }
            }
        }

        return reverse;
    }

    private static void AddReverseEntry
    (
        Dictionary<long, List<long>> reverse,
        long                         key,
        long                         value
    )
    {
        if (!reverse.TryGetValue(key, out var values))
            reverse[key] = values = [];

        if (!values.Contains(value))
            values.Add(value);
    }

    private static int GetOrCreateNode
    (
        SearchState state,
        long        refs
    )
    {
        if (state.IndexByRef.TryGetValue(refs, out var index))
            return index;

        var node = new SearchNode(refs);
        index                  = state.Nodes.Count;
        state.IndexByRef[refs] = index;
        state.Nodes.Add(node);
        return index;
    }

    private static void Push
    (
        SearchState state,
        SearchNode  node,
        float       f
    )
    {
        node.F      = f;
        node.IsOpen = true;
        node.Version++;
        state.Open.Enqueue(new HeapEntry(state.IndexByRef[node.Ref], f, node.Version), f);
    }

    private static bool TryPeekValid
    (
        SearchState   state,
        out HeapEntry entry
    )
    {
        while (state.Open.Count > 0)
        {
            var top  = state.Open.Peek();
            var node = state.Nodes[top.NodeIndex];

            if (node.IsOpen && node.Version == top.Version)
            {
                entry = top;
                return true;
            }

            state.Open.Dequeue();
        }

        entry = default;
        return false;
    }

    private static HeapEntry PopValid
    (
        SearchState state
    )
    {
        while (state.Open.Count > 0)
        {
            var entry = state.Open.Dequeue();
            var node  = state.Nodes[entry.NodeIndex];
            if (node.IsOpen && node.Version == entry.Version)
                return entry;
        }

        return default;
    }

    private static bool TryComputePortalIntersection
    (
        DtNavMeshQuery meshQuery,
        RcVec3f        rayStart,
        long           fromRef,
        RcVec3f        rayEnd,
        long           toRef,
        out RcVec3f    point
    )
    {
        point = RcVec3f.Zero;

        if (meshQuery.GetPortalPoints(fromRef, toRef, out var left, out var right, out _, out _).Failed())
            return false;

        var t = 0.5f;
        if (DtUtils.IntersectSegSeg2D(rayStart, rayEnd, left, right, out var rayParameter, out var portalParameter) &&
            rayParameter    >= 0f && rayParameter    <= 1f &&
            portalParameter >= 0f && portalParameter <= 1f)
            t = Math.Clamp(portalParameter, 0.1f, 0.9f);

        point = RcVec3f.Lerp(left, right, t);
        return true;
    }

    private static void ExpandForward
    (
        DtNavMeshQuery meshQuery,
        DtNavMesh      mesh,
        IDtQueryFilter filter,
        float          heuristicScale,
        SearchState    forward,
        SearchState    backward,
        int            nodeIndex,
        long           endRef,
        RcVec3f        endPos,
        MeetingResult  meeting
    )
    {
        var node = forward.Nodes[nodeIndex];
        mesh.GetTileAndPolyByRefUnsafe(node.Ref, out var tile, out var poly);

        long       prevRef  = 0;
        DtMeshTile prevTile = null;
        DtPoly     prevPoly = null;

        if (node.ParentIndex >= 0)
        {
            var parent = forward.Nodes[node.ParentIndex];
            prevRef = parent.Ref;
            mesh.GetTileAndPolyByRefUnsafe(prevRef, out prevTile, out prevPoly);
        }

        for (var linkIndex = poly.firstLink; linkIndex != DT_NULL_LINK; linkIndex = tile.links[linkIndex].next)
        {
            var neighbourRef = tile.links[linkIndex].refs;
            if (neighbourRef == 0 || neighbourRef == prevRef)
                continue;

            mesh.GetTileAndPolyByRefUnsafe(neighbourRef, out var neighbourTile, out var neighbourPoly);
            if (!filter.PassFilter(neighbourRef, neighbourTile, neighbourPoly))
                continue;

            var neighbourIndex = GetOrCreateNode(forward, neighbourRef);
            var neighbour      = forward.Nodes[neighbourIndex];

            if (!neighbour.IsOpen && !neighbour.IsClosed)
            {
                if (!TryComputePortalIntersection(meshQuery, node.Pos, node.Ref, endPos, neighbourRef, out var entryPos))
                    continue;

                neighbour.Pos = entryPos;
            }

            var edgeCost = filter.GetCost
            (
                node.Pos,
                neighbour.Pos,
                prevRef,
                prevTile,
                prevPoly,
                node.Ref,
                tile,
                poly,
                neighbourRef,
                neighbourTile,
                neighbourPoly
            );
            var newG = node.G + edgeCost;

            float heuristic;

            if (neighbourRef == endRef)
            {
                var endCost = filter.GetCost
                (
                    neighbour.Pos,
                    endPos,
                    node.Ref,
                    tile,
                    poly,
                    neighbourRef,
                    neighbourTile,
                    neighbourPoly,
                    0,
                    null,
                    null
                );
                newG      += endCost;
                heuristic =  0;
            }
            else
                heuristic = heuristicScale * RcVec3f.Distance(neighbour.Pos, endPos);

            if (neighbour.IsOpen && newG >= neighbour.G)
                continue;

            if (neighbour.IsClosed && newG >= neighbour.G)
                continue;

            neighbour.G           = newG;
            neighbour.F           = newG + heuristic;
            neighbour.ParentIndex = nodeIndex;
            neighbour.IsClosed    = false;
            neighbour.IsOpen      = true;
            neighbour.Version++;
            forward.Open.Enqueue(new HeapEntry(neighbourIndex, neighbour.F, neighbour.Version), neighbour.F);

            if (backward.IndexByRef.TryGetValue(neighbourRef, out var backwardIndex))
            {
                var backwardNode = backward.Nodes[backwardIndex];
                if (backwardNode.IsClosed)
                    TryRecordMeeting
                    (
                        mesh,
                        filter,
                        forward,
                        neighbourIndex,
                        backward,
                        backwardIndex,
                        endRef,
                        meeting
                    );
            }
        }
    }

    private static void ExpandBackward
    (
        DtNavMeshQuery               meshQuery,
        DtNavMesh                    mesh,
        IDtQueryFilter               filter,
        float                        heuristicScale,
        SearchState                  forward,
        SearchState                  backward,
        Dictionary<long, List<long>> reverseAdjacency,
        int                          nodeIndex,
        long                         endRef,
        RcVec3f                      startPos,
        RcVec3f                      endPos,
        MeetingResult                meeting
    )
    {
        var node = backward.Nodes[nodeIndex];
        mesh.GetTileAndPolyByRefUnsafe(node.Ref, out var tile, out var poly);

        if (!filter.PassFilter(node.Ref, tile, poly))
            return;

        long parentRef = 0;
        if (node.ParentIndex >= 0)
            parentRef = backward.Nodes[node.ParentIndex].Ref;

        void Relax
        (
            long predecessorRef
        )
        {
            if (predecessorRef == 0 || predecessorRef == parentRef)
                return;

            mesh.GetTileAndPolyByRefUnsafe(predecessorRef, out var predecessorTile, out var predecessorPoly);
            if (!filter.PassFilter(predecessorRef, predecessorTile, predecessorPoly))
                return;

            var predecessorIndex = GetOrCreateNode(backward, predecessorRef);
            var predecessor      = backward.Nodes[predecessorIndex];

            if (!predecessor.IsOpen && !predecessor.IsClosed)
            {
                if (!TryComputePortalIntersection(meshQuery, node.Pos, predecessorRef, startPos, node.Ref, out var entryPos))
                    return;

                predecessor.Pos = predecessorPoly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION ?
                                      GetOppositeOffMeshVertex(predecessorTile, predecessorPoly, entryPos) :
                                      entryPos;
            }

            var edgeCost = filter.GetCost
            (
                predecessor.Pos,
                node.Pos,
                0,
                null,
                null,
                predecessorRef,
                predecessorTile,
                predecessorPoly,
                node.Ref,
                tile,
                poly
            );
            var newG = node.G + edgeCost;

            if (node.Ref == endRef)
            {
                var endCost = filter.GetCost
                (
                    node.Pos,
                    endPos,
                    predecessorRef,
                    predecessorTile,
                    predecessorPoly,
                    node.Ref,
                    tile,
                    poly,
                    0,
                    null,
                    null
                );
                newG += endCost;
            }

            var heuristic = heuristicScale * RcVec3f.Distance(predecessor.Pos, startPos);

            if (predecessor.IsOpen && newG >= predecessor.G)
                return;

            if (predecessor.IsClosed && newG >= predecessor.G)
                return;

            predecessor.G           = newG;
            predecessor.F           = newG + heuristic;
            predecessor.ParentIndex = nodeIndex;
            predecessor.IsClosed    = false;
            predecessor.IsOpen      = true;
            predecessor.Version++;
            backward.Open.Enqueue(new HeapEntry(predecessorIndex, predecessor.F, predecessor.Version), predecessor.F);

            if (forward.IndexByRef.TryGetValue(predecessorRef, out var forwardIndex))
            {
                var forwardNode = forward.Nodes[forwardIndex];
                if (forwardNode.IsClosed)
                    TryRecordMeeting
                    (
                        mesh,
                        filter,
                        forward,
                        forwardIndex,
                        backward,
                        predecessorIndex,
                        endRef,
                        meeting
                    );
            }
        }

        if (poly.GetPolyType() == DtPolyTypes.DT_POLYTYPE_OFFMESH_CONNECTION)
        {
            if (reverseAdjacency.TryGetValue(node.Ref, out var incoming))
            {
                foreach (var predecessorRef in incoming)
                    Relax(predecessorRef);
            }

            return;
        }

        for (var linkIndex = poly.firstLink; linkIndex != DT_NULL_LINK; linkIndex = tile.links[linkIndex].next)
            Relax(tile.links[linkIndex].refs);

        if (reverseAdjacency.TryGetValue(node.Ref, out var offMeshIncoming))
        {
            foreach (var predecessorRef in offMeshIncoming)
                Relax(predecessorRef);
        }
    }

    private static RcVec3f GetOppositeOffMeshVertex
    (
        DtMeshTile tile,
        DtPoly     poly,
        RcVec3f    shared
    )
    {
        var first  = poly.verts[0] * 3;
        var second = poly.verts[1] * 3;
        var v0     = new RcVec3f(tile.data.verts[first],  tile.data.verts[first  + 1], tile.data.verts[first  + 2]);
        var v1     = new RcVec3f(tile.data.verts[second], tile.data.verts[second + 1], tile.data.verts[second + 2]);
        return RcVec3f.DistanceSquared(v0, shared) >= RcVec3f.DistanceSquared(v1, shared) ?
                   v0 :
                   v1;
    }

    private static void TryRecordMeeting
    (
        DtNavMesh      mesh,
        IDtQueryFilter filter,
        SearchState    forward,
        int            forwardIndex,
        SearchState    backward,
        int            backwardIndex,
        long           endRef,
        MeetingResult  meeting
    )
    {
        var forwardNode  = forward.Nodes[forwardIndex];
        var backwardNode = backward.Nodes[backwardIndex];
        var bridgeCost   = 0f;

        if (forwardNode.Ref != endRef)
        {
            mesh.GetTileAndPolyByRefUnsafe(forwardNode.Ref, out var tile, out var poly);

            long       previousRef  = 0;
            DtMeshTile previousTile = null;
            DtPoly     previousPoly = null;

            if (forwardNode.ParentIndex >= 0)
            {
                var previous = forward.Nodes[forwardNode.ParentIndex];
                previousRef = previous.Ref;
                mesh.GetTileAndPolyByRefUnsafe(previousRef, out previousTile, out previousPoly);
            }

            long       nextRef  = 0;
            DtMeshTile nextTile = null;
            DtPoly     nextPoly = null;

            if (backwardNode.ParentIndex >= 0)
            {
                var next = backward.Nodes[backwardNode.ParentIndex];
                nextRef = next.Ref;
                mesh.GetTileAndPolyByRefUnsafe(nextRef, out nextTile, out nextPoly);
            }

            bridgeCost = filter.GetCost
            (
                forwardNode.Pos,
                backwardNode.Pos,
                previousRef,
                previousTile,
                previousPoly,
                forwardNode.Ref,
                tile,
                poly,
                nextRef,
                nextTile,
                nextPoly
            );
        }

        var candidate = forwardNode.G + bridgeCost + backwardNode.G;
        if (candidate >= meeting.BestPath)
            return;

        meeting.BestPath      = candidate;
        meeting.ForwardIndex  = forwardIndex;
        meeting.BackwardIndex = backwardIndex;
    }

    private static void BuildCorridor
    (
        SearchState forward,
        SearchState backward,
        int         meetForward,
        int         meetBackward,
        List<long>  corridor
    )
    {
        var forwardChain = new List<long>();
        for (var index = meetForward; index >= 0; index = forward.Nodes[index].ParentIndex)
            forwardChain.Add(forward.Nodes[index].Ref);
        forwardChain.Reverse();

        var backwardChain = new List<long>();
        for (var index = meetBackward; index >= 0; index = backward.Nodes[index].ParentIndex)
            backwardChain.Add(backward.Nodes[index].Ref);

        corridor.AddRange(forwardChain);
        for (var index = 1; index < backwardChain.Count; ++index)
            corridor.Add(backwardChain[index]);
    }

    private static void BuildPartialCorridor
    (
        SearchState forward,
        int         nodeIndex,
        List<long>  corridor
    )
    {
        var chain = new List<long>();
        for (var index = nodeIndex; index >= 0; index = forward.Nodes[index].ParentIndex)
            chain.Add(forward.Nodes[index].Ref);
        chain.Reverse();
        corridor.AddRange(chain);
    }

    private sealed class SearchState
    {
        public List<SearchNode> Nodes { get; } = [];

        public Dictionary<long, int> IndexByRef { get; } = [];

        public PriorityQueue<HeapEntry, float> Open { get; } = new();
    }

    private sealed class SearchNode
    (
        long refs
    )
    {
        public long Ref { get; } = refs;

        public RcVec3f Pos { get; set; }

        public float G { get; set; }

        public float F { get; set; }

        public int ParentIndex { get; set; } = -1;

        public int Version { get; set; }

        public bool IsOpen { get; set; }

        public bool IsClosed { get; set; }
    }

    private sealed class MeetingResult
    {
        public float BestPath { get; set; } = float.PositiveInfinity;

        public int ForwardIndex { get; set; } = -1;

        public int BackwardIndex { get; set; } = -1;
    }

    private readonly record struct PathQuality
    (
        float Length,
        int   CornerCount,
        float TurnAmount,
        bool  IsValid
    )
    {
        public float Score =>
            Length                                                   +
            (CornerCount * GEOMETRY_REFINEMENT_CORNER_PENALTY)       +
            (TurnAmount * GEOMETRY_REFINEMENT_TURN_PENALTY);
    }

    private readonly record struct HeapEntry
    (
        int   NodeIndex,
        float F,
        int   Version
    );
}
