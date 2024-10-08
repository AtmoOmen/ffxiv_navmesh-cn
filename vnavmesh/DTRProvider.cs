﻿using Dalamud.Game.Gui.Dtr;
using System;

namespace Navmesh;

public class DTRProvider : IDisposable
{
    private NavmeshManager _manager;
    private IDtrBarEntry _dtrBarEntry;

    public DTRProvider(NavmeshManager manager)
    {
        _manager = manager;
        _dtrBarEntry = Service.DtrBar.Get("vnavmesh");
    }

    public void Dispose()
    {
        _dtrBarEntry.Remove();
    }

    public void Update()
    {
        _dtrBarEntry.Shown = Service.Config.EnableDTR;
        if (_dtrBarEntry.Shown)
        {
            var loadProgress = _manager.LoadTaskProgress;
            var status = loadProgress >= 0 ? $"{loadProgress * 100:f0}%" : _manager.Navmesh != null ? "就绪" : "加载中";
            _dtrBarEntry.Text = "导航状态: " + status;
        }
    }
}
