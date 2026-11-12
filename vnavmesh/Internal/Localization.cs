using Lumina.Data;

namespace vnavmesh.Internal;

internal readonly struct Localization
(
    Language language
)
{
    public static readonly Language[] Languages =
    [
        Language.ChineseSimplified,
        Language.English,
        Language.Japanese,
        Language.German,
        Language.French,
        Language.TraditionalChinese,
        Language.Korean
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

    public string AutoLoadNavmesh => Select
    (
        chineseSimplified: "切换区域时自动加载导航",
        english: "Load navigation data when changing zones",
        japanese: "エリア移動時にナビゲーションデータを自動読み込み",
        german: "Navigationsdaten beim Gebietswechsel laden",
        french: "Charger les données de navigation au changement de zone",
        chineseTraditional: "切換區域時自動載入導航",
        korean: "지역 변경 시 탐색 데이터 자동 로드"
    );

    public string AutoLoadNavmeshHelp => Select
    (
        chineseSimplified: "切换区域时，自动构建导航数据或加载已有的缓存数据。",
        english: "Automatically build navigation data or load existing cached data when changing zones.",
        japanese: "エリア移動時に、ナビゲーションデータを自動で構築するか、既存のキャッシュを読み込みます。",
        german: "Beim Gebietswechsel automatisch Navigationsdaten erstellen oder vorhandene Cache-Daten laden.",
        french: "Générer automatiquement les données de navigation ou charger le cache existant au changement de zone.",
        chineseTraditional: "切換區域時，自動建置導航資料或載入已有的快取資料。",
        korean: "지역 변경 시 탐색 데이터를 자동으로 생성하거나 기존 캐시 데이터를 로드합니다."
    );

    public string CacheRetentionDays => Select
    (
        chineseSimplified: "旧缓存保留时间（天）",
        english: "Old cache retention (days)",
        japanese: "古いキャッシュの保持日数",
        german: "Aufbewahrungsdauer alter Caches (Tage)",
        french: "Durée de conservation du cache (jours)",
        chineseTraditional: "舊快取保留時間（天）",
        korean: "오래된 캐시 보관 기간 (일)"
    );

    public string CacheRetentionDaysHelp => Select
    (
        chineseSimplified: "任一新导航数据构建完成后，会检查并移除超过该时长未使用的导航缓存；设为0表示保留全部缓存。",
        english: "After any new navigation data is built, check for and remove navigation caches unused for longer than this period. Set to 0 to keep all caches.",
        japanese: "新しいナビゲーションデータの構築が完了するたびに、指定日数を超えて使用していないキャッシュを確認して削除します。0 にするとすべて保持します。",
        german:
        "Nach jeder Erstellung neuer Navigationsdaten prüfen, welche Navigationscaches länger als angegeben ungenutzt sind, und diese entfernen. 0 behält alle Caches.",
        french:
        "Après chaque génération de nouvelles données de navigation, rechercher et supprimer les caches inutilisés depuis plus longtemps que cette durée. 0 conserve tous les caches.",
        chineseTraditional: "任一新導航資料建置完成後，會檢查並移除超過該時長未使用的導航快取；設為 0 表示保留全部快取。",
        korean: "새 탐색 데이터 생성이 완료될 때마다 지정한 기간보다 오래 사용하지 않은 탐색 캐시를 확인하고 삭제합니다. 0으로 설정하면 모두 보관합니다."
    );

    public string ShowWaypoints => Select
    (
        chineseSimplified: "显示路径",
        english: "Show path",
        japanese: "経路を表示",
        german: "Pfad anzeigen",
        french: "Afficher le chemin",
        chineseTraditional: "顯示路徑",
        korean: "경로 표시"
    );

    public string ShowWaypointsHelp => Select
    (
        chineseSimplified: "寻路过程中，绘制本次寻路的路径。",
        english: "Draw the current path while navigating.",
        japanese: "経路に沿って移動している間、現在の経路を描画します。",
        german: "Während der Navigation den aktuellen Pfad zeichnen.",
        french: "Afficher le chemin actuel pendant la navigation.",
        chineseTraditional: "尋路過程中，繪製本次尋路的路徑。",
        korean: "경로를 따라 이동하는 동안 현재 경로를 표시합니다."
    );

    public string RenderWhenGameUIHidden => Select
    (
        chineseSimplified: "隐藏游戏界面时仍然渲染",
        english: "Render while the game UI is hidden",
        japanese: "ゲーム UI 非表示時も描画",
        german: "Bei ausgeblendeter Spieloberfläche weiter rendern",
        french: "Afficher même si l’interface du jeu est masquée",
        chineseTraditional: "隱藏遊戲介面時仍然繪製",
        korean: "게임 UI를 숨겨도 렌더링"
    );

    public string RenderWhenGameUIHiddenHelp => Select
    (
        chineseSimplified: "隐藏游戏界面时，仍然继续渲染插件绘制的界面元素。",
        english: "Keep rendering the plugin’s visual elements when the game UI is hidden.",
        japanese: "ゲーム UI を非表示にしても、プラグインの表示要素を描画し続けます。",
        german: "Die Anzeigeelemente des Plugins auch bei ausgeblendeter Spieloberfläche weiter zeichnen.",
        french: "Continuer à afficher les éléments visuels du plugin lorsque l’interface du jeu est masquée.",
        chineseTraditional: "隱藏遊戲介面時，仍然繼續繪製外掛的介面元素。",
        korean: "게임 UI가 숨겨져 있어도 플러그인의 화면 요소를 계속 렌더링합니다."
    );

    public string EnableDTR => Select
    (
        chineseSimplified: "添加“基本信息”条目",
        english: "Add a Server Info entry",
        japanese: "サーバー情報に項目を追加",
        german: "Eintrag zur Serverinfo hinzufügen",
        french: "Ajouter une entrée aux informations serveur",
        chineseTraditional: "新增「基本資訊」項目",
        korean: "서버 정보에 항목 추가"
    );

    public string EnableDTRHelp => Select
    (
        chineseSimplified: "在“基本信息”界面内添加会显示插件信息的条目。",
        english: "Add an entry displaying plugin information to the Server Info UI element.",
        japanese: "サーバー情報の UI に、プラグインの情報を表示する項目を追加します。",
        german: "Der Serverinfo ein Anzeigeelement mit Informationen zum Plugin hinzufügen.",
        french: "Ajouter aux informations serveur une entrée affichant les informations du plugin.",
        chineseTraditional: "在「基本資訊」介面內新增顯示外掛資訊的項目。",
        korean: "서버 정보 UI에 플러그인 정보를 표시하는 항목을 추가합니다."
    );

    public string CancelMoveOnUserInput => Select
    (
        chineseSimplified: "手动移动时取消寻路",
        english: "Cancel navigation when moving manually",
        japanese: "手動移動時にナビゲーションをキャンセル",
        german: "Navigation bei manueller Bewegung abbrechen",
        french: "Annuler la navigation lors d’un déplacement manuel",
        chineseTraditional: "手動移動時取消尋路",
        korean: "직접 이동 시 경로 이동 취소"
    );

    public string CancelMoveOnUserInputHelp => Select
    (
        chineseSimplified: "手动移动角色时，立刻取消当前正在进行的寻路任务。",
        english: "Immediately cancel the current navigation task when moving the character manually.",
        japanese: "キャラクターを手動で移動すると、実行中のナビゲーションを直ちにキャンセルします。",
        german: "Die aktuelle Navigation sofort abbrechen, sobald der Charakter manuell bewegt wird.",
        french: "Annuler immédiatement la navigation en cours lorsque le personnage est déplacé manuellement.",
        chineseTraditional: "手動移動角色時，立刻取消目前正在進行的尋路任務。",
        korean: "캐릭터를 직접 이동하면 진행 중인 경로 이동 작업을 즉시 취소합니다."
    );

    public string PathTolerance => Select
    (
        chineseSimplified: "默认终点容差",
        english: "Default destination tolerance",
        japanese: "到着判定の既定の許容距離",
        german: "Standard-Zieltoleranz",
        french: "Tolérance à destination par défaut",
        chineseTraditional: "預設終點容差",
        korean: "기본 목적지 허용 거리"
    );

    public string PathToleranceHelp => Select
    (
        chineseSimplified: "当距离目标点不足该设定值时，视为已经到达目的地并结束寻路。",
        english: "When the distance to the target is less than this value, consider the destination reached and end navigation.",
        japanese: "目標地点までの距離が設定値より短くなると、到着したと判定してナビゲーションを終了します。",
        german: "Ist die Entfernung zum Ziel kleiner als dieser Wert, gilt das Ziel als erreicht und die Navigation endet.",
        french: "Lorsque la distance à la cible est inférieure à cette valeur, considérer la destination atteinte et terminer la navigation.",
        chineseTraditional: "當距離目標點不足該設定值時，視為已經到達目的地並結束尋路。",
        korean: "목표 지점까지의 거리가 설정값보다 작아지면 목적지에 도착한 것으로 간주하고 경로 이동을 종료합니다."
    );

    public string AlignCameraToMovement => Select
    (
        chineseSimplified: "镜头跟随寻路移动方向",
        english: "Face the camera in the navigation movement direction",
        japanese: "ナビゲーション中の移動方向にカメラを向ける",
        german: "Kamera während der Navigation in Bewegungsrichtung ausrichten",
        french: "Orienter la caméra dans le sens du déplacement pendant la navigation",
        chineseTraditional: "鏡頭跟隨尋路移動方向",
        korean: "경로 이동 방향으로 카메라 회전"
    );

    public string AlignCameraToMovementHelp => Select
    (
        chineseSimplified: "寻路时，自动将镜头转向角色当前的移动方向。",
        english: "During navigation, automatically turn the camera toward the character’s current movement direction.",
        japanese: "ナビゲーション中、キャラクターの現在の移動方向にカメラを自動で向けます。",
        german: "Während der Navigation die Kamera automatisch in die aktuelle Bewegungsrichtung des Charakters drehen.",
        french: "Pendant la navigation, orienter automatiquement la caméra dans le sens du déplacement actuel du personnage.",
        chineseTraditional: "尋路時，自動將鏡頭轉向角色目前的移動方向。",
        korean: "경로 이동 중 캐릭터의 현재 이동 방향으로 카메라를 자동 회전합니다."
    );

    public string AlignCameraHeight => Select
    (
        chineseSimplified: "镜头高度角",
        english: "Camera elevation",
        japanese: "カメラの仰角",
        german: "Kameraneigung",
        french: "Inclinaison de la caméra",
        chineseTraditional: "鏡頭高度角",
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
        chineseSimplified: "处理角色卡死",
        english: "Recover when the character is stuck",
        japanese: "キャラクターのスタックに対処",
        german: "Feststeckenden Charakter befreien",
        french: "Débloquer le personnage",
        chineseTraditional: "處理角色卡住",
        korean: "캐릭터 끼임 해제"
    );

    public string StopOnStuckHelp => Select
    (
        chineseSimplified: "寻路时，当检测到角色几乎未能有效移动一段时间后，自动尝试跳跃、朝着另一个方向移动和重新算路以脱困。",
        english:
        "During navigation, if the character makes almost no progress for a while, automatically try jumping, moving in another direction, and recalculating the path to get unstuck.",
        japanese: "ナビゲーション中、一定時間ほとんど移動できていないことを検出すると、ジャンプ、別方向への移動、経路の再計算で自動的に復帰を試みます。",
        german:
        "Kommt der Charakter während der Navigation eine Zeit lang kaum voran, automatisch durch Springen, Bewegung in eine andere Richtung und Neuberechnung des Pfads eine Befreiung versuchen.",
        french:
        "Pendant la navigation, si le personnage ne progresse presque plus pendant un certain temps, essayer automatiquement de sauter, de changer de direction et de recalculer le chemin pour le débloquer.",
        chineseTraditional: "尋路時，當偵測到角色一段時間內幾乎未能有效移動後，自動嘗試跳躍、朝另一個方向移動和重新計算路徑以脫困。",
        korean: "경로 이동 중 일정 시간 동안 캐릭터가 거의 이동하지 못하면 점프, 다른 방향으로 이동, 경로 재계산을 자동으로 시도하여 끼임을 해제합니다."
    );

    public string UnstuckDetectionSeconds => Select
    (
        chineseSimplified: "卡死判定时长（秒）",
        english: "Stuck detection time (seconds)",
        japanese: "スタック判定時間（秒）",
        german: "Stillstandserkennung (Sekunden)",
        french: "Délai de détection du blocage (secondes)",
        chineseTraditional: "卡住判定時長（秒）",
        korean: "끼임 감지 시간 (초)"
    );

    public string UnstuckDetectionSecondsHelp => Select
    (
        chineseSimplified: "当角色在设定时间内都未能有效移动，认定角色卡死。",
        english: "Consider the character stuck if it makes no meaningful movement for this duration.",
        japanese: "設定した時間の間、キャラクターがほとんど移動できない場合にスタックと判定します。",
        german: "Den Charakter als feststeckend einstufen, wenn er sich für diese Dauer nicht nennenswert bewegt.",
        french: "Considérer le personnage comme bloqué s’il ne se déplace pas de manière significative pendant cette durée.",
        chineseTraditional: "當角色在設定時間內都未能有效移動，判定角色卡住。",
        korean: "설정한 시간 동안 캐릭터가 유의미하게 이동하지 못하면 끼임 상태로 판단합니다."
    );

    public string UnstuckCooldownSeconds => Select
    (
        chineseSimplified: "尝试脱困间隔（秒）",
        english: "Recovery attempt interval (seconds)",
        japanese: "復帰試行の間隔（秒）",
        german: "Intervall zwischen Befreiungsversuchen (Sekunden)",
        french: "Intervalle entre les tentatives de déblocage (secondes)",
        chineseTraditional: "嘗試脫困間隔（秒）",
        korean: "끼임 해제 시도 간격 (초)"
    );

    public string UnstuckCooldownSecondsHelp => Select
    (
        chineseSimplified: "每次尝试脱困后，都会暂停卡死检测一段时间，然后再继续检测是否卡死。",
        english: "After each recovery attempt, pause stuck detection for this duration before checking again.",
        japanese: "復帰を試みるたびに、設定した時間だけスタック検出を停止してから、再び検出を行います。",
        german: "Nach jedem Befreiungsversuch die Stillstandserkennung für diese Dauer aussetzen und anschließend fortsetzen.",
        french: "Après chaque tentative de déblocage, suspendre la détection du blocage pendant cette durée avant de la reprendre.",
        chineseTraditional: "每次嘗試脫困後，都會暫停卡住偵測一段時間，然後再繼續偵測是否卡住。",
        korean: "끼임 해제를 시도할 때마다 설정한 시간 동안 끼임 감지를 일시 중지한 뒤 다시 감지합니다."
    );

    public string CommandHelp => Select
    (
        chineseSimplified: """
                           /vnav 或 /vnavmesh：打开或关闭插件窗口；以下子命令均支持这两个命令名。
                           /vnav moveto <X> <Y> <Z>：寻路移动到指定世界坐标。
                           /vnav movedir <X> <Y> <Z>：以当前位置为起点，按世界坐标轴偏移指定距离后寻路移动。
                           /vnav movetarget：寻路移动到当前目标的位置。
                           /vnav moveflag：寻路移动到地图旗标的位置。
                           /vnav flyto <X> <Y> <Z>：飞行寻路到指定世界坐标。
                           /vnav flydir <X> <Y> <Z>：以当前位置为起点，按世界坐标轴偏移指定距离后飞行寻路。
                           /vnav flytarget：飞行寻路到当前目标的位置。
                           /vnav flyflag：飞行寻路到地图旗标的位置。
                           坐标和偏移量均按 X、Y、Z 顺序填写，Y 表示高度；小数使用英文句点。
                           /vnav stop：停止当前移动。
                           /vnav reload：重新加载当前区域的导航数据，优先使用缓存。
                           /vnav rebuild：重新构建当前区域的导航数据，不使用缓存。
                           /vnav aligncamera：切换镜头跟随寻路移动方向的开关。
                           /vnav aligncamera true|yes|enable：启用镜头跟随；任选一个参数。
                           /vnav aligncamera false|no|disable：禁用镜头跟随；任选一个参数。
                           /vnav dtr：显示或隐藏插件的“基本信息”条目。
                           """,
        english: """
                 /vnav or /vnavmesh: Toggle the plugin window. Both command names support all subcommands below.
                 /vnav moveto <X> <Y> <Z>: Navigate to the specified world coordinates.
                 /vnav movedir <X> <Y> <Z>: Navigate to the current position plus the specified world-axis offsets.
                 /vnav movetarget: Navigate to the current target’s position.
                 /vnav moveflag: Navigate to the map flag’s position.
                 /vnav flyto <X> <Y> <Z>: Navigate by flight to the specified world coordinates.
                 /vnav flydir <X> <Y> <Z>: Navigate by flight to the current position plus the specified world-axis offsets.
                 /vnav flytarget: Navigate by flight to the current target’s position.
                 /vnav flyflag: Navigate by flight to the map flag’s position.
                 Enter coordinates and offsets in X, Y, Z order; Y is height. Use a period as the decimal separator.
                 /vnav stop: Stop the current movement.
                 /vnav reload: Reload navigation data for the current zone, using the cache when available.
                 /vnav rebuild: Rebuild navigation data for the current zone without using the cache.
                 /vnav aligncamera: Toggle camera alignment with the navigation movement direction.
                 /vnav aligncamera true|yes|enable: Enable camera alignment; choose one argument.
                 /vnav aligncamera false|no|disable: Disable camera alignment; choose one argument.
                 /vnav dtr: Show or hide the plugin’s Server Info entry.
                 """,
        japanese: """
                  /vnav または /vnavmesh：プラグインのウィンドウを開閉します。以下のサブコマンドはどちらのコマンド名でも使用できます。
                  /vnav moveto <X> <Y> <Z>：指定したワールド座標まで経路に沿って移動します。
                  /vnav movedir <X> <Y> <Z>：現在位置にワールド座標軸で指定した変位を加えた地点まで移動します。
                  /vnav movetarget：現在のターゲットの位置まで移動します。
                  /vnav moveflag：地図のフラッグの位置まで移動します。
                  /vnav flyto <X> <Y> <Z>：指定したワールド座標まで飛行経路に沿って移動します。
                  /vnav flydir <X> <Y> <Z>：現在位置にワールド座標軸で指定した変位を加えた地点まで飛行移動します。
                  /vnav flytarget：現在のターゲットの位置まで飛行移動します。
                  /vnav flyflag：地図のフラッグの位置まで飛行移動します。
                  座標と変位は X、Y、Z の順に指定します。Y は高さです。小数点には半角のピリオドを使用します。
                  /vnav stop：現在の移動を停止します。
                  /vnav reload：現在のエリアのナビゲーションデータを、キャッシュがあれば使用して再読み込みします。
                  /vnav rebuild：キャッシュを使用せず、現在のエリアのナビゲーションデータを再構築します。
                  /vnav aligncamera：ナビゲーション中の移動方向へのカメラ追従を切り替えます。
                  /vnav aligncamera true|yes|enable：カメラ追従を有効にします。いずれか 1 つを指定します。
                  /vnav aligncamera false|no|disable：カメラ追従を無効にします。いずれか 1 つを指定します。
                  /vnav dtr：サーバー情報にあるプラグインの項目の表示を切り替えます。
                  """,
        german: """
                /vnav oder /vnavmesh: Plugin-Fenster öffnen oder schließen. Beide Befehlsnamen unterstützen alle folgenden Unterbefehle.
                /vnav moveto <X> <Y> <Z>: Zu den angegebenen Weltkoordinaten navigieren.
                /vnav movedir <X> <Y> <Z>: Zur aktuellen Position zuzüglich der angegebenen Versätze entlang der Weltachsen navigieren.
                /vnav movetarget: Zur Position des aktuellen Ziels navigieren.
                /vnav moveflag: Zur Position der Kartenmarkierung navigieren.
                /vnav flyto <X> <Y> <Z>: Im Flug zu den angegebenen Weltkoordinaten navigieren.
                /vnav flydir <X> <Y> <Z>: Im Flug zur aktuellen Position zuzüglich der angegebenen Versätze entlang der Weltachsen navigieren.
                /vnav flytarget: Im Flug zur Position des aktuellen Ziels navigieren.
                /vnav flyflag: Im Flug zur Position der Kartenmarkierung navigieren.
                Koordinaten und Versätze in der Reihenfolge X, Y, Z angeben; Y ist die Höhe. Einen Punkt als Dezimaltrennzeichen verwenden.
                /vnav stop: Die aktuelle Bewegung stoppen.
                /vnav reload: Navigationsdaten des aktuellen Gebiets neu laden und dabei den Cache verwenden, sofern vorhanden.
                /vnav rebuild: Navigationsdaten des aktuellen Gebiets ohne Cache neu erstellen.
                /vnav aligncamera: Die Ausrichtung der Kamera in Bewegungsrichtung während der Navigation umschalten.
                /vnav aligncamera true|yes|enable: Kameraausrichtung aktivieren; ein Argument auswählen.
                /vnav aligncamera false|no|disable: Kameraausrichtung deaktivieren; ein Argument auswählen.
                /vnav dtr: Den Serverinfo-Eintrag des Plugins ein- oder ausblenden.
                """,
        french: """
                /vnav ou /vnavmesh : ouvrir ou fermer la fenêtre du plugin. Les deux noms acceptent toutes les sous-commandes suivantes.
                /vnav moveto <X> <Y> <Z> : suivre un chemin vers les coordonnées du monde indiquées.
                /vnav movedir <X> <Y> <Z> : suivre un chemin vers la position actuelle augmentée des décalages indiqués selon les axes du monde.
                /vnav movetarget : suivre un chemin vers la position de la cible actuelle.
                /vnav moveflag : suivre un chemin vers la position du drapeau sur la carte.
                /vnav flyto <X> <Y> <Z> : suivre un itinéraire de vol vers les coordonnées du monde indiquées.
                /vnav flydir <X> <Y> <Z> : suivre un itinéraire de vol vers la position actuelle augmentée des décalages indiqués selon les axes du monde.
                /vnav flytarget : suivre un itinéraire de vol vers la position de la cible actuelle.
                /vnav flyflag : suivre un itinéraire de vol vers la position du drapeau sur la carte.
                Saisir les coordonnées et décalages dans l’ordre X, Y, Z ; Y représente la hauteur. Utiliser un point comme séparateur décimal.
                /vnav stop : arrêter le déplacement en cours.
                /vnav reload : recharger les données de navigation de la zone actuelle en utilisant le cache s’il est disponible.
                /vnav rebuild : régénérer les données de navigation de la zone actuelle sans utiliser le cache.
                /vnav aligncamera : activer ou désactiver l’orientation de la caméra dans le sens du déplacement pendant la navigation.
                /vnav aligncamera true|yes|enable : activer le suivi de la caméra ; choisir un seul argument.
                /vnav aligncamera false|no|disable : désactiver le suivi de la caméra ; choisir un seul argument.
                /vnav dtr : afficher ou masquer l’entrée du plugin dans les informations serveur.
                """,
        chineseTraditional: """
                            /vnav 或 /vnavmesh：開啟或關閉外掛視窗；以下子命令均支援這兩個命令名稱。
                            /vnav moveto <X> <Y> <Z>：尋路移動到指定世界座標。
                            /vnav movedir <X> <Y> <Z>：以目前位置為起點，沿世界座標軸偏移指定距離後尋路移動。
                            /vnav movetarget：尋路移動到目前目標的位置。
                            /vnav moveflag：尋路移動到地圖旗標的位置。
                            /vnav flyto <X> <Y> <Z>：飛行尋路到指定世界座標。
                            /vnav flydir <X> <Y> <Z>：以目前位置為起點，沿世界座標軸偏移指定距離後飛行尋路。
                            /vnav flytarget：飛行尋路到目前目標的位置。
                            /vnav flyflag：飛行尋路到地圖旗標的位置。
                            座標和偏移量均按 X、Y、Z 順序填寫，Y 表示高度；小數使用半形句點。
                            /vnav stop：停止目前移動。
                            /vnav reload：重新載入目前區域的導航資料，優先使用快取。
                            /vnav rebuild：重新建置目前區域的導航資料，不使用快取。
                            /vnav aligncamera：切換鏡頭跟隨尋路移動方向的開關。
                            /vnav aligncamera true|yes|enable：啟用鏡頭跟隨；任選一個參數。
                            /vnav aligncamera false|no|disable：停用鏡頭跟隨；任選一個參數。
                            /vnav dtr：顯示或隱藏外掛的「基本資訊」項目。
                            """,
        korean: """
                /vnav 또는 /vnavmesh: 플러그인 창을 열거나 닫습니다. 아래 하위 명령은 두 명령 이름 모두에서 사용할 수 있습니다.
                /vnav moveto <X> <Y> <Z>: 지정한 월드 좌표까지 경로를 따라 이동합니다.
                /vnav movedir <X> <Y> <Z>: 현재 위치에 월드 좌표축 기준 이동량을 더한 지점까지 경로를 따라 이동합니다.
                /vnav movetarget: 현재 대상의 위치까지 경로를 따라 이동합니다.
                /vnav moveflag: 지도 깃발의 위치까지 경로를 따라 이동합니다.
                /vnav flyto <X> <Y> <Z>: 지정한 월드 좌표까지 비행 경로를 따라 이동합니다.
                /vnav flydir <X> <Y> <Z>: 현재 위치에 월드 좌표축 기준 이동량을 더한 지점까지 비행 경로를 따라 이동합니다.
                /vnav flytarget: 현재 대상의 위치까지 비행 경로를 따라 이동합니다.
                /vnav flyflag: 지도 깃발의 위치까지 비행 경로를 따라 이동합니다.
                좌표와 이동량은 X, Y, Z 순서로 입력하며 Y는 높이입니다. 소수점은 마침표를 사용합니다.
                /vnav stop: 현재 이동을 중지합니다.
                /vnav reload: 현재 지역의 탐색 데이터를 다시 로드하며, 캐시가 있으면 사용합니다.
                /vnav rebuild: 캐시를 사용하지 않고 현재 지역의 탐색 데이터를 다시 생성합니다.
                /vnav aligncamera: 경로 이동 방향으로 카메라를 회전하는 기능을 전환합니다.
                /vnav aligncamera true|yes|enable: 카메라 따라가기를 켭니다. 인수 중 하나를 선택합니다.
                /vnav aligncamera false|no|disable: 카메라 따라가기를 끕니다. 인수 중 하나를 선택합니다.
                /vnav dtr: 서버 정보에 있는 플러그인 항목을 표시하거나 숨깁니다.
                """
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
