using System.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using DotRecast.Core.Numerics;
using DotRecast.Detour;

namespace vnavmesh.Query.Ground;

using static DtDetour;

internal static class BidirectionalGroundPathSearch
{
    private const int   MAX_CORRIDOR_POLYS                 = 4096;
    private const int   MIN_GEOMETRY_REFINEMENT_CORRIDOR   = 32;
    private const int   MAX_GEOMETRY_REFINEMENT_POINTS     = 4097;
    private const float GEOMETRY_REFINEMENT_MARGIN         = 0.25f;
    private const float GEOMETRY_REFINEMENT_CORNER_PENALTY = 0.20f;
    private const float GEOMETRY_REFINEMENT_TURN_PENALTY   = 2.00f;

    private static readonly ConditionalWeakTable<DtNavMesh, Dictionary<long, List<long>>> ReverseAdjacency = new();

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

        var mesh = meshQuery.GetAttachedNavMesh();
        if (!mesh.IsValidPolyRef(startRef)  ||
            !mesh.IsValidPolyRef(endRef)    ||
            !startPos.IsFinite()            ||
            !endPos.IsFinite()              ||
            !float.IsFinite(heuristicScale) ||
            heuristicScale < 0)
            return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;

        if (startRef == endRef)
        {
            corridor.Add(startRef);
            return DtStatus.DT_SUCCESS;
        }

        var context  = new SearchContext(meshQuery, filter, startRef, endRef, startPos, endPos, heuristicScale);
        var forward  = context.Forward;
        var backward = context.Backward;

        var forwardStartIndex = GetOrCreateNode(forward, 0, startRef, startPos);
        var forwardStart      = forward.Nodes[forwardStartIndex];
        Push(forward, forwardStart, context.SourceTargetEstimate);

        var backwardEndIndex = GetOrCreateNode(backward, endRef, 0, endPos);
        var backwardEnd      = backward.Nodes[backwardEndIndex];
        Push(backward, backwardEnd, context.SourceTargetEstimate);

        var meeting            = context.Meeting;
        var bestPartialForward = 0;
        var bestPartialScore   = RcVec3f.Distance(startPos, endPos);

        while (true)
        {
            var hasForward  = TryPeekValid(forward,  out var forwardTop);
            var hasBackward = TryPeekValid(backward, out var backwardTop);

            if (!hasForward && !hasBackward)
                break;

            if (hasForward && hasBackward)
            {
                if (forwardTop.F + backwardTop.F >= meeting.BestPath + context.SourceTargetEstimate)
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

        if (meeting is { ForwardIndex: >= 0, BackwardIndex: >= 0 })
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

            var partialScore = RcVec3f.Distance(node.Pos, endPos);

            if (node.Ref != 0 && partialScore < bestPartialScore)
            {
                bestPartialScore   = partialScore;
                bestPartialForward = entry.NodeIndex;
            }

            ExpandForward(context, entry.NodeIndex);
        }

        void ExpandBackwardSide()
        {
            var entry = PopValid(backward);
            var node  = backward.Nodes[entry.NodeIndex];
            node.IsOpen   = false;
            node.IsClosed = true;

            ExpandBackward(context, entry.NodeIndex);
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
                heuristicScale == 0f ?
                    DtFindPathOption.ZeroScale :
                    DtFindPathOption.NoOption
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
            var outgoingX      = next.X    - current.X;
            var outgoingZ      = next.Z    - current.Z;
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
            if (tile?.data?.header == null || tile.links == null)
                continue;

            var baseRef = mesh.GetPolyRefBase(tile);

            for (var polyIndex = 0; polyIndex < tile.data.header.polyCount; ++polyIndex)
            {
                var poly    = tile.data.polys[polyIndex];
                var polyRef = baseRef | (uint)polyIndex;

                for (var linkIndex = poly.firstLink; linkIndex != DT_NULL_LINK; linkIndex = tile.links[linkIndex].next)
                {
                    var link = tile.links[linkIndex];
                    if (link.refs == 0)
                        continue;

                    AddReverseEntry(reverse, link.refs, polyRef);
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
        long        fromRef,
        long        refs,
        RcVec3f     position
    )
    {
        if (state.IndexByPortal.TryGetValue((fromRef, refs), out var index))
            return index;

        var node = new SearchNode(fromRef, refs, position);
        index                                = state.Nodes.Count;
        state.IndexByPortal[(fromRef, refs)] = index;
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
        state.Open.Enqueue(new HeapEntry(state.IndexByPortal[(node.FromRef, node.Ref)], f, node.Version), f);
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

        point = ResolvePortalPoint(rayStart, rayEnd, left, right);
        return true;
    }

    internal static RcVec3f ResolvePortalPoint
    (
        RcVec3f rayStart,
        RcVec3f rayEnd,
        RcVec3f left,
        RcVec3f right
    )
    {
        if (DtUtils.IntersectSegSeg2D(rayStart, rayEnd, left, right, out var rayParameter, out var portalParameter) &&
            rayParameter is >= 0f and <= 1f                                                                         &&
            portalParameter is >= 0f and <= 1f)
            return RcVec3f.Lerp(left, right, portalParameter);

        DtUtils.DistancePtSegSqr2D(rayStart, left, right, out var startParameter);
        DtUtils.DistancePtSegSqr2D(rayEnd,   left, right, out var endParameter);
        Span<float> parameters      = [startParameter, endParameter, 0f, 1f];
        var         nearestDistance = float.PositiveInfinity;
        var         result          = left;

        foreach (var parameter in parameters)
        {
            var candidate = RcVec3f.Lerp(left, right, parameter);
            var distance  = DtUtils.DistancePtSegSqr2D(candidate, rayStart, rayEnd, out _);
            if (distance >= nearestDistance)
                continue;

            nearestDistance = distance;
            result          = candidate;
        }

        return result;
    }

    private static void ExpandForward
    (
        SearchContext context,
        int           nodeIndex
    )
    {
        var node = context.Forward.Nodes[nodeIndex];
        if (node.Ref == 0)
            return;

        context.Mesh.GetTileAndPolyByRefUnsafe(node.Ref, out var tile, out var poly);
        if (!context.Filter.PassFilter(node.Ref, tile, poly))
            return;

        if (node.Ref == context.EndRef)
        {
            RelaxTransition(context, nodeIndex, node.FromRef, node.Ref, 0, false);
            return;
        }

        for (var linkIndex = poly.firstLink; linkIndex != DT_NULL_LINK; linkIndex = tile.links[linkIndex].next)
        {
            var neighbourRef = tile.links[linkIndex].refs;
            if (neighbourRef == 0 || neighbourRef == node.FromRef)
                continue;

            RelaxTransition(context, nodeIndex, node.FromRef, node.Ref, neighbourRef, false);
        }
    }

    private static void ExpandBackward
    (
        SearchContext context,
        int           nodeIndex
    )
    {
        var node = context.Backward.Nodes[nodeIndex];
        if (node.FromRef == 0)
            return;

        context.Mesh.GetTileAndPolyByRefUnsafe(node.FromRef, out var tile, out var poly);
        if (!context.Filter.PassFilter(node.FromRef, tile, poly))
            return;

        if (node.FromRef == context.StartRef)
        {
            RelaxTransition(context, nodeIndex, 0, node.FromRef, node.Ref, true);
            return;
        }

        if (!context.IncomingEdges.TryGetValue(node.FromRef, out var incoming))
            return;

        foreach (var predecessorRef in incoming)
        {
            if (predecessorRef == node.Ref)
                continue;

            RelaxTransition(context, nodeIndex, predecessorRef, node.FromRef, node.Ref, true);
        }
    }

    private static void RelaxTransition
    (
        SearchContext context,
        int           nodeIndex,
        long          previousRef,
        long          currentRef,
        long          nextRef,
        bool          reverse
    )
    {
        var active = reverse ?
                         context.Backward :
                         context.Forward;
        var opposite = reverse ?
                           context.Forward :
                           context.Backward;
        var node = active.Nodes[nodeIndex];
        var fromRef = reverse ?
                          previousRef :
                          currentRef;
        var toRef = reverse ?
                        currentRef :
                        nextRef;
        RcVec3f position;

        if (fromRef == 0)
            position = context.StartPosition;
        else if (toRef == 0)
            position = context.EndPosition;
        else if (!TryComputePortalIntersection(context.MeshQuery, context.StartPosition, fromRef, context.EndPosition, toRef, out position))
            return;

        context.Mesh.GetTileAndPolyByRefUnsafe(currentRef, out var currentTile, out var currentPoly);

        DtMeshTile previousTile = null;
        DtPoly     previousPoly = null;
        if (previousRef != 0)
            context.Mesh.GetTileAndPolyByRefUnsafe(previousRef, out previousTile, out previousPoly);

        DtMeshTile nextTile = null;
        DtPoly     nextPoly = null;
        if (nextRef != 0)
            context.Mesh.GetTileAndPolyByRefUnsafe(nextRef, out nextTile, out nextPoly);

        var adjacentRef = reverse ?
                              previousRef :
                              nextRef;
        var adjacentTile = reverse ?
                               previousTile :
                               nextTile;
        var adjacentPoly = reverse ?
                               previousPoly :
                               nextPoly;
        if (adjacentRef != 0 && !context.Filter.PassFilter(adjacentRef, adjacentTile, adjacentPoly))
            return;

        var edgeCost = context.Filter.GetCost
        (
            reverse ?
                position :
                node.Pos,
            reverse ?
                node.Pos :
                position,
            previousRef,
            previousTile,
            previousPoly,
            currentRef,
            currentTile,
            currentPoly,
            nextRef,
            nextTile,
            nextPoly
        );
        var newG           = node.G + edgeCost;
        var neighbourIndex = GetOrCreateNode(active, fromRef, toRef, position);
        var neighbour      = active.Nodes[neighbourIndex];
        if ((neighbour.IsOpen || neighbour.IsClosed) && newG >= neighbour.G)
            return;

        neighbour.G           = newG;
        neighbour.ParentIndex = nodeIndex;
        neighbour.IsClosed    = false;

        var distanceFromStart = RcVec3f.Distance(position, context.StartPosition);
        var distanceToEnd     = RcVec3f.Distance(position, context.EndPosition);
        var potential = reverse ?
                            distanceFromStart - distanceToEnd :
                            distanceToEnd     - distanceFromStart;
        var heuristic = ((context.HeuristicScale * potential) + context.SourceTargetEstimate) * 0.5f;
        Push(active, neighbour, newG + heuristic);

        if (!opposite.IndexByPortal.TryGetValue((fromRef, toRef), out var oppositeIndex))
            return;

        var oppositeNode = opposite.Nodes[oppositeIndex];
        var candidate    = newG + oppositeNode.G;
        if (candidate >= context.Meeting.BestPath)
            return;

        context.Meeting.BestPath = candidate;
        context.Meeting.ForwardIndex = reverse ?
                                           oppositeIndex :
                                           neighbourIndex;
        context.Meeting.BackwardIndex = reverse ?
                                            neighbourIndex :
                                            oppositeIndex;
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

        foreach (var reference in forwardChain)
        {
            if (reference != 0)
                corridor.Add(reference);
        }

        for (var index = 1; index < backwardChain.Count; ++index)
        {
            if (backwardChain[index] != 0)
                corridor.Add(backwardChain[index]);
        }
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

    private sealed class SearchContext
    (
        DtNavMeshQuery meshQuery,
        IDtQueryFilter filter,
        long           startRef,
        long           endRef,
        RcVec3f        startPosition,
        RcVec3f        endPosition,
        float          heuristicScale
    )
    {
        public DtNavMeshQuery MeshQuery { get; } = meshQuery;

        public DtNavMesh Mesh { get; } = meshQuery.GetAttachedNavMesh();

        public IDtQueryFilter Filter { get; } = filter;

        public long StartRef { get; } = startRef;

        public long EndRef { get; } = endRef;

        public RcVec3f StartPosition { get; } = startPosition;

        public RcVec3f EndPosition { get; } = endPosition;

        public float HeuristicScale { get; } = heuristicScale;

        public float SourceTargetEstimate { get; } = heuristicScale * RcVec3f.Distance(startPosition, endPosition);

        public Dictionary<long, List<long>> IncomingEdges { get; } = ReverseAdjacency.GetValue(meshQuery.GetAttachedNavMesh(), BuildReverseAdjacency);

        public SearchState Forward { get; } = new();

        public SearchState Backward { get; } = new();

        public MeetingResult Meeting { get; } = new();
    }

    private sealed class SearchState
    {
        public List<SearchNode> Nodes { get; } = [];

        public Dictionary<(long FromRef, long Ref), int> IndexByPortal { get; } = [];

        public PriorityQueue<HeapEntry, float> Open { get; } = new();
    }

    private sealed class SearchNode
    (
        long    fromRef,
        long    refs,
        RcVec3f position
    )
    {
        public long FromRef { get; } = fromRef;

        public long Ref { get; } = refs;

        public RcVec3f Pos { get; } = position;

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
            Length                                             +
            (CornerCount * GEOMETRY_REFINEMENT_CORNER_PENALTY) +
            (TurnAmount  * GEOMETRY_REFINEMENT_TURN_PENALTY);
    }

    private readonly record struct HeapEntry
    (
        int   NodeIndex,
        float F,
        int   Version
    );
}
