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

## 直角版 `OrthogonalFromLengths`

全頂点を**厳密に 90°**（直交多角形／折れ線）にし、各辺長も**厳密**に保つ別ソルバ。基準角 θ は任意（概略点から推定、または基準辺の方向で指定）。

全角 90° ＋ 全辺長厳密なら図形は「基準角 θ ＋ 開始点 ＋ 各頂点の曲がり向き(±90°)」で一意に決まるため、**反復不要**。曲がり向きは概略点から、θ は概略点への 2D プロクラステス（閉形式）で求める。

> 直交「閉合」多角形は辺数が偶数で θ方向／θ+90°方向の辺の和がそれぞれ 0 でないと閉じない。「全辺長厳密」＋「全角90°」＋「閉合」は一般に同時に満たせないため、本ソルバは**閉合を強制せず**、`closed: true` のときは閉合差 `ClosureGap` を返す（開チェーンとして配置）。

```csharp
// 概略4点 + 4辺長 → 全角90°・辺長厳密(θは推定)。Points は P0..Pn(末尾=戻り点)、ClosureGap で閉合差
var r = OrthogonalFromLengths.Solve(approx, lengths, closed: true);

// 基準辺を指定: edge0 を水平(θ=0)に固定
var r2 = OrthogonalFromLengths.Solve(approx, lengths, closed: true, baseAngleRad: 0.0, alignEdgeIndex: 0);
```

`Result`: 頂点 `Points`（open は n 点、closed は n+1 点で末尾が始点への戻り点）、基準角 `BaseAngleRad`、閉合差 `ClosureGap`、概略点へのRMS残差 `FitResidual`、各頂点の曲がり向き `TurnSigns`、直角判定が曖昧だったか `HasAmbiguousCorner`。

## ビルド・テスト

```
dotnet run --project GeomSolver.Tests
```

依存ゼロの簡易テストランナー（辺長版＋直角版あわせて 30 件：三角形 SSS、四角形、長方形不動、開放折れ線、頂点固定、自己交差検出、成立しない辺長、直角化の回転復元・基準辺指定・厳密辺長 など）。NuGet 復元不要・オフライン可。

## 構成

```
GeomSolver/
├─ GeomSolver/
│   ├─ PolylineFromLengths.cs     … 辺長拘束ソルバ + 自己交差検査
│   └─ OrthogonalFromLengths.cs   … 全頂点90°・辺長厳密ソルバ
└─ GeomSolver.Tests/              … 依存ゼロのテストランナー（30件）
```
