# GeomSolver

概略の頂点位置（クリック点）と各辺の指定長から、**辺長拘束をちょうど満たし、かつ概略位置からの移動量が最小**になる多角形／折れ線を求める再利用ライブラリ。測量CADの「おおよそ N 点を指定 ＋ 各辺長を指定して図形を作る」操作のための幾何ソルバ。

- 依存ゼロ・`net10.0` クラスライブラリ（`PolylineFromLengths.cs` 1ファイルをコピーするだけで組み込み可）
- N角形汎用 / 閉合（closed）・開放（open）切替
- 頂点ごとの重み（`weights`）で頂点固定（既知点拘束）
- 自己交差検出・閉合差（残差）の報告

## なぜ「概略点」が要るのか

辺の長さだけでは図形は一意に決まらない。三角形（3辺）は SSS で一意だが、四角形以上は四節リンクのように変形できる自由度が残る（閉合多角形で内部自由度 = `n - 3`）。この残った自由度と複数ありうる解の枝を、ユーザーがクリックした概略点で決める、というのがこのソルバの考え方。

## アルゴリズム

クリック点 `Q_i` を初期値・目標として、

```
minimize  Σ w_i · ‖P_i − Q_i‖²      （概略点からの移動量を最小化）
s.t.      ‖P_{i+1} − P_i‖ = L_i      （各辺長をちょうど満たす）
```

を、辺長²拘束の KKT 系（目的関数のヘッセ行列＝重み対角）に対するガウス・ニュートン反復で解く。初期値が概略点なので数回で収束。

## 使い方

```csharp
using Geometry;

// 閉合：おおよそ4点 + 4辺長 → 閉じた四角形
var quad = PolylineFromLengths.Solve(
    new[]{ new Vec2(0,0), new Vec2(10,1), new Vec2(11,8), new Vec2(1,9) },
    new[]{ 10.0, 7.0, 10.0, 9.0 },   // 閉合 → 辺数 = 頂点数
    closed: true);

// 開放：5点の折れ線、辺長は4本
var open = PolylineFromLengths.Solve(pts5, len4, closed: false);  // 辺数 = 頂点数 - 1

// 頂点固定：P0 を強く固定
var pinned = PolylineFromLengths.Solve(approx, lengths, closed: true,
    weights: new[]{ 1e6, 1.0, 1.0, 1.0 });
```

`Result` は補正後頂点 `Points`、辺ごとの誤差 `EdgeError`、最大誤差／閉合差 `MaxError`、収束フラグ `Converged`、自己交差 `SelfIntersects` / `Crossings` を返す。

## ビルド・テスト

```
dotnet run --project GeomSolver.Tests
```

依存ゼロの簡易テストランナー（三角形 SSS、四角形、長方形不動、開放折れ線、頂点固定、自己交差検出、成立しない辺長 など 15 件）。NuGet 復元不要・オフライン可。

## 構成

```
GeomSolver/
├─ GeomSolver/            … クラスライブラリ（PolylineFromLengths.cs）
└─ GeomSolver.Tests/      … 依存ゼロのテストランナー
```
