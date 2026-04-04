using System.Runtime.InteropServices;

namespace vnavmesh.Models;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ExdZoneSharedGroup
{
    uint LGBSharedGroup;
    uint RequirementRow0;
    uint RequirementRow1;
    uint RequirementRow2;
    uint RequirementRow3;
    uint RequirementRow4;
    uint RequirementRow5;
    uint Unknown0;
    uint RequirementQuestSequence0;
    uint RequirementQuestSequence1;
    uint RequirementQuestSequence2;
    uint RequirementQuestSequence3;
    uint RequirementQuestSequence4;
    uint RequirementQuestSequence5;
    uint Unknown1;
    byte RequirementType0;
    byte RequirementType1;
    byte RequirementType2;
    byte RequirementType3;
    byte RequirementType4;
    byte RequirementType5;
    byte Unknown8;
    byte Unknown9;
    byte Unknown10;
    byte Unknown11;
    byte Unknown12;
    byte Unknown13;
    byte Unknown14;
    byte Unknown15;

    public static implicit operator ExdZoneSharedGroup(Lumina.Excel.Sheets.ZoneSharedGroup sg) => new()
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
        Unknown9                  = sg.Unknown9 ? (byte)1 : (byte)0,
        Unknown10                 = sg.Unknown10 ? (byte)1 : (byte)0,
        Unknown11                 = sg.Unknown11 ? (byte)1 : (byte)0,
        Unknown12                 = sg.Unknown12 ? (byte)1 : (byte)0,
        Unknown13                 = sg.Unknown13 ? (byte)1 : (byte)0,
        Unknown14                 = sg.Unknown14 ? (byte)1 : (byte)0,
        Unknown15                 = sg.Unknown15 ? (byte)1 : (byte)0,
    };
}
