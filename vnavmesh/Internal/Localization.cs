using Lumina.Data;

namespace vnavmesh.Internal;

internal readonly struct Localization
(
    Language language
)
{
    public static readonly Language[] Languages =
    [
        Language.ChineseSimplified, Language.English, Language.Japanese, Language.German,
        Language.French, Language.TraditionalChinese, Language.Korean
    ];

    public static readonly string[] LanguageNames =
    [
        "简体中文", 
        "English", 
        "日本語", 
        "Deutsch",
        "Français", 
        "繁體中文", 
        "한국어"
    ];

    public int LanguageIndex => language switch
    {
        Language.ChineseSimplified or Language.ChineseTraditional => 0,
        Language.English                                          => 1,
        Language.Japanese                                         => 2,
        Language.German                                           => 3,
        Language.French                                           => 4,
        Language.TraditionalChinese                               => 5,
        Language.Korean                                           => 6,
        _                                                         => throw new ArgumentOutOfRangeException(nameof(language), language, null)
    };

    public string LanguageLabel => Select
    (
        chineseSimplified: "语言",
        english: "Language",
        japanese: "言語",
        german: "Sprache",
        french: "Langue",
        chineseTraditional: "語言",
        korean: "언어"
    );

    public string NavigationAndDisplay => Select
    (
        chineseSimplified: "导航与显示",
        english: "Navigation and display",
        japanese: "ナビゲーションと表示",
        german: "Navigation und Anzeige",
        french: "Navigation et affichage",
        chineseTraditional: "導航與顯示",
        korean: "탐색 및 표시"
    );

    public string NavigationAndDisplayDescription => Select
    (
        chineseSimplified: "控制导航数据加载、可视化以及 DTR 信息栏展示。",
        english: "Configure navigation data loading, visualization and the DTR bar.",
        japanese: "ナビゲーションデータの読み込み、可視化、DTR バーの表示を設定します。",
        german: "Laden der Navigationsdaten, Visualisierung und DTR-Leiste konfigurieren.",
        french: "Configurer le chargement des données de navigation, la visualisation et la barre DTR.",
        chineseTraditional: "控制導航資料載入、視覺化以及 DTR 資訊列顯示。",
        korean: "탐색 데이터 로드, 시각화 및 DTR 표시줄을 설정합니다."
    );

    public string MovementAndCamera => Select
    (
        chineseSimplified: "移动与镜头",
        english: "Movement and camera",
        japanese: "移動とカメラ",
        german: "Bewegung und Kamera",
        french: "Déplacement et caméra",
        chineseTraditional: "移動與鏡頭",
        korean: "이동 및 카메라"
    );

    public string MovementAndCameraDescription => Select
    (
        chineseSimplified: "调整角色跟随行为、镜头联动与寻路细节。",
        english: "Adjust movement behavior, camera tracking and pathfinding.",
        japanese: "キャラクターの移動、カメラの追従、経路探索を調整します。",
        german: "Bewegungsverhalten, Kameraführung und Wegfindung anpassen.",
        french: "Ajuster le déplacement, le suivi de la caméra et la recherche de chemin.",
        chineseTraditional: "調整角色跟隨行為、鏡頭連動與尋路細節。",
        korean: "캐릭터 이동, 카메라 추적 및 경로 탐색을 조정합니다."
    );

    public string StuckHandling => Select
    (
        chineseSimplified: "卡住处理",
        english: "Stuck recovery",
        japanese: "スタック対策",
        german: "Befreiung bei Stillstand",
        french: "Déblocage",
        chineseTraditional: "卡住處理",
        korean: "끼임 해제"
    );

    public string StuckHandlingDescription => Select
    (
        chineseSimplified: "配置角色疑似卡住时的判定条件与恢复策略。",
        english: "Configure stuck detection and recovery.",
        japanese: "スタックの判定条件と復帰動作を設定します。",
        german: "Erkennung und Behebung von Stillstand konfigurieren.",
        french: "Configurer la détection de blocage et le déblocage.",
        chineseTraditional: "設定角色疑似卡住時的判定條件與恢復策略。",
        korean: "끼임 감지 조건과 해제 동작을 설정합니다."
    );

    public string AutoLoadNavmesh => Select
    (
        chineseSimplified: "切换区域时自动加载导航数据",
        english: "Load navigation data when changing zones",
        japanese: "エリア移動時にナビゲーションデータを自動読み込み",
        german: "Navigationsdaten beim Gebietswechsel laden",
        french: "Charger les données de navigation au changement de zone",
        chineseTraditional: "切換區域時自動載入導航資料",
        korean: "지역 변경 시 탐색 데이터 자동 로드"
    );

    public string AutoLoadNavmeshHelp => Select
    (
        chineseSimplified: "进入新区域后自动尝试载入对应的导航数据。",
        english: "Automatically load the navigation data for each new zone.",
        japanese: "新しいエリアに入ると、対応するナビゲーションデータを自動で読み込みます。",
        german: "Beim Betreten eines neuen Gebiets die zugehörigen Navigationsdaten automatisch laden.",
        french: "Charger automatiquement les données de navigation de chaque nouvelle zone.",
        chineseTraditional: "進入新區域後自動嘗試載入對應的導航資料。",
        korean: "새 지역에 진입하면 해당 탐색 데이터를 자동으로 로드합니다."
    );

    public string CacheRetentionDays => Select
    (
        chineseSimplified: "自动移除旧缓存（天）",
        english: "Remove old cache after (days)",
        japanese: "古いキャッシュの保持日数",
        german: "Alten Cache entfernen nach (Tagen)",
        french: "Supprimer le cache après (jours)",
        chineseTraditional: "自動移除舊快取（天）",
        korean: "오래된 캐시 삭제 기준 (일)"
    );

    public string CacheRetentionDaysHelp => Select
    (
        chineseSimplified: "超过该时长未使用的区域缓存会在构建完成后被移除；设为 0 表示保留全部缓存。",
        english: "Remove zone caches unused for this many days after a build completes. Set to 0 to keep all caches.",
        japanese: "ビルド完了後、指定日数以上使用していないエリアのキャッシュを削除します。0 にするとすべて保持します。",
        german: "Nach Abschluss einer Berechnung Gebietscaches entfernen, die so viele Tage ungenutzt sind. 0 behält alle Caches.",
        french: "Après une génération, supprimer les caches inutilisés depuis ce nombre de jours. 0 conserve tous les caches.",
        chineseTraditional: "超過該時長未使用的區域快取會在建置完成後被移除；設為 0 表示保留全部快取。",
        korean: "생성이 완료되면 지정한 일수 동안 사용하지 않은 지역 캐시를 삭제합니다. 0으로 설정하면 모두 보관합니다."
    );

    public string ShowWaypoints => Select
    (
        chineseSimplified: "显示路径点",
        english: "Show waypoints",
        japanese: "経由点を表示",
        german: "Wegpunkte anzeigen",
        french: "Afficher les points de passage",
        chineseTraditional: "顯示路徑點",
        korean: "경유 지점 표시"
    );

    public string ShowWaypointsHelp => Select
    (
        chineseSimplified: "在世界中绘制当前路径的关键点。",
        english: "Draw the waypoints of the current path in the world.",
        japanese: "現在の経路の経由点をワールド内に描画します。",
        german: "Wegpunkte des aktuellen Pfads in der Spielwelt zeichnen.",
        french: "Afficher les points de passage du chemin actuel dans le monde.",
        chineseTraditional: "在世界中繪製目前路徑的關鍵點。",
        korean: "현재 경로의 경유 지점을 게임 공간에 표시합니다."
    );

    public string ForceShowGameCollision => Select
    (
        chineseSimplified: "始终显示游戏碰撞体积",
        english: "Always show game collision",
        japanese: "ゲームの当たり判定を常に表示",
        german: "Spielkollisionen immer anzeigen",
        french: "Toujours afficher les collisions du jeu",
        chineseTraditional: "始終顯示遊戲碰撞體積",
        korean: "게임 충돌 영역 항상 표시"
    );

    public string ForceShowGameCollisionHelp => Select
    (
        chineseSimplified: "用于排查地形碰撞与导航结果之间的差异。",
        english: "Inspect differences between terrain collision and navigation results.",
        japanese: "地形の当たり判定とナビゲーション結果の違いを確認します。",
        german: "Unterschiede zwischen Geländekollisionen und Navigationsergebnissen untersuchen.",
        french: "Examiner les différences entre les collisions du terrain et les résultats de navigation.",
        chineseTraditional: "用於排查地形碰撞與導航結果之間的差異。",
        korean: "지형 충돌과 탐색 결과의 차이를 확인합니다."
    );

    public string RenderWhenGameUIHidden => Select
    (
        chineseSimplified: "游戏隐藏界面时仍然渲染",
        english: "Render while the game UI is hidden",
        japanese: "ゲーム UI 非表示時も描画",
        german: "Bei ausgeblendeter Spieloberfläche weiter rendern",
        french: "Afficher même si l’interface du jeu est masquée",
        chineseTraditional: "遊戲隱藏介面時仍然繪製",
        korean: "게임 UI를 숨겨도 렌더링"
    );

    public string RenderWhenGameUIHiddenHelp => Select
    (
        chineseSimplified: "即使使用游戏内隐藏界面功能，插件的调试渲染与窗口仍继续绘制。",
        english: "Keep drawing plugin windows and debug visuals when the game UI is hidden.",
        japanese: "ゲーム内で UI を非表示にしても、プラグインのウィンドウとデバッグ表示を描画します。",
        german: "Plugin-Fenster und Debug-Anzeigen auch bei ausgeblendeter Spieloberfläche zeichnen.",
        french: "Continuer à afficher les fenêtres du plugin et les éléments de débogage lorsque l’interface du jeu est masquée.",
        chineseTraditional: "即使使用遊戲內隱藏介面功能，外掛的偵錯繪製與視窗仍繼續顯示。",
        korean: "게임 UI가 숨겨져 있어도 플러그인 창과 디버그 표시를 계속 그립니다."
    );

    public string EnableDTR => Select
    (
        chineseSimplified: "启用 DTR 信息栏",
        english: "Enable DTR bar",
        japanese: "DTR バーを有効化",
        german: "DTR-Leiste aktivieren",
        french: "Activer la barre DTR",
        chineseTraditional: "啟用 DTR 資訊列",
        korean: "DTR 표시줄 사용"
    );

    public string EnableDTRHelp => Select
    (
        chineseSimplified: "在界面上方的信息栏显示插件状态。",
        english: "Show plugin status in the information bar at the top of the screen.",
        japanese: "画面上部の情報バーにプラグインの状態を表示します。",
        german: "Plugin-Status in der Informationsleiste am oberen Bildschirmrand anzeigen.",
        french: "Afficher l’état du plugin dans la barre d’informations en haut de l’écran.",
        chineseTraditional: "在介面上方的資訊列顯示外掛狀態。",
        korean: "화면 상단 정보 표시줄에 플러그인 상태를 표시합니다."
    );

    public string ShowQueryStatusInDTR => Select
    (
        chineseSimplified: "在 DTR 信息栏中显示详细查询状态",
        english: "Show detailed query status in the DTR bar",
        japanese: "DTR バーに詳細なクエリ状態を表示",
        german: "Detaillierten Abfragestatus in der DTR-Leiste anzeigen",
        french: "Afficher le détail des requêtes dans la barre DTR",
        chineseTraditional: "在 DTR 資訊列中顯示詳細查詢狀態",
        korean: "DTR 표시줄에 상세 요청 상태 표시"
    );

    public string ShowQueryStatusInDTRHelp => Select
    (
        chineseSimplified: "显示更细的查询状态变化，便于观察插件当前行为。",
        english: "Show detailed query status changes to monitor current activity.",
        japanese: "クエリ状態の変化を詳しく表示し、現在の動作を確認しやすくします。",
        german: "Detaillierte Änderungen des Abfragestatus zur Überwachung der aktuellen Aktivität anzeigen.",
        french: "Afficher les changements détaillés des requêtes pour suivre l’activité actuelle.",
        chineseTraditional: "顯示更詳細的查詢狀態變化，便於觀察外掛目前的行為。",
        korean: "현재 동작을 확인할 수 있도록 요청 상태 변화를 자세히 표시합니다."
    );

    public string CancelMoveOnUserInput => Select
    (
        chineseSimplified: "玩家产生移动输入时取消当前路径",
        english: "Cancel the current path on movement input",
        japanese: "移動入力時に現在の経路をキャンセル",
        german: "Aktuellen Pfad bei Bewegungseingabe abbrechen",
        french: "Annuler le chemin actuel lors d’une commande de déplacement",
        chineseTraditional: "玩家產生移動輸入時取消目前路徑",
        korean: "이동 입력 시 현재 경로 취소"
    );

    public string CancelMoveOnUserInputHelp => Select
    (
        chineseSimplified: "手动接管角色时立即停止自动移动。",
        english: "Stop automatic movement immediately when moving manually.",
        japanese: "手動で移動すると、自動移動を直ちに停止します。",
        german: "Automatische Bewegung bei manueller Steuerung sofort stoppen.",
        french: "Arrêter immédiatement le déplacement automatique lors d’un déplacement manuel.",
        chineseTraditional: "手動接管角色時立即停止自動移動。",
        korean: "직접 이동하면 자동 이동을 즉시 중지합니다."
    );

    public string PathTolerance => Select
    (
        chineseSimplified: "终点容差",
        english: "Destination tolerance",
        japanese: "到着判定の許容距離",
        german: "Zieltoleranz",
        french: "Tolérance à destination",
        chineseTraditional: "終點容差",
        korean: "목적지 허용 거리"
    );

    public string PathToleranceHelp => Select
    (
        chineseSimplified: "角色进入目标点附近该范围后，视为已经到达最终目的地。",
        english: "Consider the destination reached when the character enters this range.",
        japanese: "目標地点からこの距離以内に入ると、目的地に到着したと判定します。",
        german: "Das Ziel gilt als erreicht, sobald sich der Charakter in dieser Entfernung befindet.",
        french: "Considérer la destination atteinte lorsque le personnage entre dans ce rayon.",
        chineseTraditional: "角色進入目標點附近該範圍後，視為已經到達最終目的地。",
        korean: "캐릭터가 목표 지점의 이 범위 안에 들어오면 도착한 것으로 간주합니다."
    );

    public string CameraTracking => Select
    (
        chineseSimplified: "镜头联动",
        english: "Camera tracking",
        japanese: "カメラ追従",
        german: "Kameraführung",
        french: "Suivi de la caméra",
        chineseTraditional: "鏡頭連動",
        korean: "카메라 추적"
    );

    public string AlignCameraToMovement => Select
    (
        chineseSimplified: "镜头跟随移动方向",
        english: "Face the camera in the movement direction",
        japanese: "カメラを移動方向に向ける",
        german: "Kamera in Bewegungsrichtung ausrichten",
        french: "Orienter la caméra dans le sens du déplacement",
        chineseTraditional: "鏡頭跟隨移動方向",
        korean: "카메라를 이동 방향으로 회전"
    );

    public string AlignCameraToMovementHelp => Select
    (
        chineseSimplified: "自动将镜头转向角色当前的移动方向。",
        english: "Automatically turn the camera toward the character’s movement direction.",
        japanese: "キャラクターの移動方向にカメラを自動で向けます。",
        german: "Die Kamera automatisch in die Bewegungsrichtung des Charakters drehen.",
        french: "Orienter automatiquement la caméra dans le sens du déplacement du personnage.",
        chineseTraditional: "自動將鏡頭轉向角色目前的移動方向。",
        korean: "캐릭터가 이동하는 방향으로 카메라를 자동 회전합니다."
    );

    public string AlignCameraHeight => Select
    (
        chineseSimplified: "相机高度角",
        english: "Camera elevation",
        japanese: "カメラの仰角",
        german: "Kameraneigung",
        french: "Inclinaison de la caméra",
        chineseTraditional: "相機高度角",
        korean: "카메라 고도각"
    );

    public string AlignCameraHeightHelp => Select
    (
        chineseSimplified: "数值越高越偏向俯视，数值越低越偏向平视。",
        english: "Higher values look down more; lower values give a more level view.",
        japanese: "値が大きいほど見下ろす角度になり、小さいほど水平に近づきます。",
        german: "Höhere Werte blicken stärker nach unten, niedrigere Werte eher waagerecht.",
        french: "Les valeurs élevées donnent une vue plongeante, les valeurs basses une vue plus horizontale.",
        chineseTraditional: "數值越高越偏向俯視，數值越低越偏向平視。",
        korean: "값이 높을수록 내려다보며, 낮을수록 수평에 가까워집니다."
    );

    public string StopOnStuck => Select
    (
        chineseSimplified: "启用自动防卡",
        english: "Enable automatic stuck recovery",
        japanese: "スタックからの自動復帰を有効化",
        german: "Automatische Befreiung aktivieren",
        french: "Activer le déblocage automatique",
        chineseTraditional: "啟用自動防卡",
        korean: "자동 끼임 해제 사용"
    );

    public string StopOnStuckHelp => Select
    (
        chineseSimplified: "检测到角色长时间无法推进路径时，自动尝试跳跃或短时脱困。",
        english: "Try jumping or brief recovery movements when the character stops making progress.",
        japanese: "経路上で長時間進めない場合、ジャンプや短い移動で復帰を試みます。",
        german: "Bei längerem Stillstand durch Springen oder kurze Bewegungen eine Befreiung versuchen.",
        french: "Essayer de sauter ou de se dégager brièvement lorsque le personnage ne progresse plus.",
        chineseTraditional: "偵測到角色長時間無法推進路徑時，自動嘗試跳躍或短暫脫困。",
        korean: "캐릭터가 오랫동안 경로를 따라 진행하지 못하면 점프나 짧은 이동으로 해제를 시도합니다."
    );

    public string StuckHandlingDisabled => Select
    (
        chineseSimplified: "启用后可进一步配置卡住判定时长与冷却期。",
        english: "Enable to configure the detection time and recovery cooldown.",
        japanese: "有効にすると、スタック判定時間と復帰の待機時間を設定できます。",
        german: "Aktivieren, um Erkennungsdauer und Abklingzeit einzustellen.",
        french: "Activer pour configurer le délai de détection et le temps de recharge du déblocage.",
        chineseTraditional: "啟用後可進一步設定卡住判定時長與冷卻期。",
        korean: "사용하면 감지 시간과 해제 재사용 대기시간을 설정할 수 있습니다."
    );

    public string UnstuckDetectionSeconds => Select
    (
        chineseSimplified: "卡住判定时长 (秒)",
        english: "Stuck detection time (seconds)",
        japanese: "スタック判定時間（秒）",
        german: "Stillstandserkennung (Sekunden)",
        french: "Délai de détection du blocage (secondes)",
        chineseTraditional: "卡住判定時長 (秒)",
        korean: "끼임 감지 시간 (초)"
    );

    public string UnstuckDetectionSecondsHelp => Select
    (
        chineseSimplified: "持续几乎没有实际位移达到该时长后，开始尝试脱困。",
        english: "Start recovery after this much time with almost no movement.",
        japanese: "実際の移動がほとんどない状態がこの時間続くと、復帰を試みます。",
        german: "Nach dieser Zeit ohne nennenswerte Bewegung eine Befreiung versuchen.",
        french: "Commencer le déblocage après ce délai sans déplacement significatif.",
        chineseTraditional: "持續幾乎沒有實際位移達到該時長後，開始嘗試脫困。",
        korean: "실제 이동이 거의 없는 상태가 이 시간 동안 지속되면 해제를 시도합니다."
    );

    public string UnstuckCooldownSeconds => Select
    (
        chineseSimplified: "脱困冷却期 (秒)",
        english: "Recovery cooldown (seconds)",
        japanese: "復帰の待機時間（秒）",
        german: "Befreiungsabklingzeit (Sekunden)",
        french: "Temps de recharge du déblocage (secondes)",
        chineseTraditional: "脫困冷卻期 (秒)",
        korean: "끼임 해제 재사용 대기시간 (초)"
    );

    public string UnstuckCooldownSecondsHelp => Select
    (
        chineseSimplified: "每次随机位移脱困结束后，暂停重新判定卡住的时间。",
        english: "Wait this long after each random recovery movement before checking for stuck movement again.",
        japanese: "ランダム移動による復帰動作の後、この時間が経過するまでスタック判定を停止します。",
        german: "Nach jeder zufälligen Befreiungsbewegung so lange bis zur nächsten Stillstandsprüfung warten.",
        french: "Attendre ce délai après chaque déplacement aléatoire de déblocage avant de vérifier à nouveau le blocage.",
        chineseTraditional: "每次隨機位移脫困結束後，暫停重新判定卡住的時間。",
        korean: "무작위 이동을 통한 해제 동작이 끝난 후 다시 끼임을 감지할 때까지 기다리는 시간입니다."
    );

    public string Ready => Select
    (
        chineseSimplified: "就绪",
        english: "Ready",
        japanese: "準備完了",
        german: "Bereit",
        french: "Prêt",
        chineseTraditional: "就緒",
        korean: "준비됨"
    );

    public string NotReady => Select
    (
        chineseSimplified: "未就绪",
        english: "Not ready",
        japanese: "未準備",
        german: "Nicht bereit",
        french: "Non prêt",
        chineseTraditional: "未就緒",
        korean: "준비 안 됨"
    );

    public string Navigation => Select
    (
        chineseSimplified: "导航",
        english: "Nav",
        japanese: "ナビ",
        german: "Navigation",
        french: "Navigation",
        chineseTraditional: "導航",
        korean: "탐색"
    );

    public string Queries => Select
    (
        chineseSimplified: "算路",
        english: "Queries",
        japanese: "経路探索",
        german: "Abfragen",
        french: "Requêtes",
        chineseTraditional: "算路",
        korean: "경로 요청"
    );

    public string Queued => Select
    (
        chineseSimplified: "等待中",
        english: "Queued",
        japanese: "待機中",
        german: "Wartend",
        french: "En attente",
        chineseTraditional: "等待中",
        korean: "대기 중"
    );

    public string Pathfinding => Select
    (
        chineseSimplified: "算路中",
        english: "Pathfinding",
        japanese: "経路探索中",
        german: "Wegfindung",
        french: "Recherche de chemin",
        chineseTraditional: "算路中",
        korean: "경로 탐색 중"
    );

    public string Moving => Select
    (
        chineseSimplified: "移动中",
        english: "Moving",
        japanese: "移動中",
        german: "In Bewegung",
        french: "Déplacement",
        chineseTraditional: "移動中",
        korean: "이동 중"
    );

    private string Select
    (
        string chineseSimplified,
        string english,
        string japanese,
        string german,
        string french,
        string chineseTraditional,
        string korean
    ) => language switch
    {
        Language.ChineseSimplified or Language.ChineseTraditional => chineseSimplified,
        Language.None or Language.English                         => english,
        Language.Japanese                                         => japanese,
        Language.German                                           => german,
        Language.French                                           => french,
        Language.TraditionalChinese                               => chineseTraditional,
        Language.Korean                                           => korean,
        _                                                         => throw new InvalidOperationException($"Unsupported language: {language}")
    };
}
