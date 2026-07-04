using System.Runtime.InteropServices;
using Lumina.Excel.Sheets;

namespace vnavmesh.Shared.Models;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ExdZoneSharedGroup
{
    private uint LGBSharedGroup;
    private uint RequirementRow0;
    private uint RequirementRow1;
    private uint RequirementRow2;
    private uint RequirementRow3;
    private uint RequirementRow4;
    private uint RequirementRow5;
    private uint Unknown0;
    private uint RequirementQuestSequence0;
    private uint RequirementQuestSequence1;
    private uint RequirementQuestSequence2;
    private uint RequirementQuestSequence3;
    private uint RequirementQuestSequence4;
    private uint RequirementQuestSequence5;
    private uint Unknown1;
    private byte RequirementType0;
    private byte RequirementType1;
    private byte RequirementType2;
    private byte RequirementType3;
    private byte RequirementType4;
    private byte RequirementType5;
    private byte Unknown8;
    private byte Unknown9;
    private byte Unknown10;
    private byte Unknown11;
    private byte Unknown12;
    private byte Unknown13;
    private byte Unknown14;
    private byte Unknown15;

    public static implicit operator ExdZoneSharedGroup(ZoneSharedGroup sg) => new()
    {
        LGBSharedGroup            = sg.LGBSharedGroup,
        RequirementRow0           = sg.RequirementRow[0].RowId,
        RequirementRow1           = sg.RequirementRow[1].RowId,
        RequirementRow2           = sg.RequirementRow[2].RowId,
        RequirementRow3           = sg.RequirementRow[3].RowId,
        RequirementRow4           = sg.RequirementRow[4].RowId,
        RequirementRow5           = sg.RequirementRow[5].RowId,
        Unknown0                  = sg.Unknown0,
        RequirementQuestSequence0 = sg.RequirementQuestSequence[0],
        RequirementQuestSequence1 = sg.RequirementQuestSequence[1],
        RequirementQuestSequence2 = sg.RequirementQuestSequence[2],
        RequirementQuestSequence3 = sg.RequirementQuestSequence[3],
        RequirementQuestSequence4 = sg.RequirementQuestSequence[4],
        RequirementQuestSequence5 = sg.RequirementQuestSequence[5],
        Unknown1                  = sg.Unknown1,
        RequirementType0          = sg.RequirementType[0],
        RequirementType1          = sg.RequirementType[1],
        RequirementType2          = sg.RequirementType[2],
        RequirementType3          = sg.RequirementType[3],
        RequirementType4          = sg.RequirementType[4],
        RequirementType5          = sg.RequirementType[5],
        Unknown8                  = sg.Unknown8,
        Unknown9 = sg.Unknown9 ?
                       (byte)1 :
                       (byte)0,
        Unknown10 = sg.Unknown10 ?
                        (byte)1 :
                        (byte)0,
        Unknown11 = sg.Unknown11 ?
                        (byte)1 :
                        (byte)0,
        Unknown12 = sg.Unknown12 ?
                        (byte)1 :
                        (byte)0,
        Unknown13 = sg.Unknown13 ?
                        (byte)1 :
                        (byte)0,
        Unknown14 = sg.Unknown14 ?
                        (byte)1 :
                        (byte)0,
        Unknown15 = sg.Unknown15 ?
                        (byte)1 :
                        (byte)0
    };
}
