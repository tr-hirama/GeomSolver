using Geometry;

// 依存ゼロの簡易テストランナー。失敗時は終了コード 1。
int passed = 0, failed = 0;

void Check(string name, bool cond, string? detail = null)
{
    if (cond) { passed++; Console.WriteLine($"  PASS  {name}"); }
    else      { failed++; Console.WriteLine($"  FAIL  {name}{(detail is null ? "" : "  -> " + detail)}"); }
}

// 補正後の各辺が指定長に一致しているか
bool EdgesMatch(PolylineFromLengths.Result r, double tol = 1e-6)
{
    foreach (var e in r.EdgeError) if (e > tol) return false;
    return true;
}

double Dist(Vec2 a, Vec2 b) { double dx = a.X - b.X, dy = a.Y - b.Y; return Math.Sqrt(dx*dx + dy*dy); }

Console.WriteLine("PolylineFromLengths tests");

// 1) 三角形(SSS=3-4-5): 概略点を少しずらしても辺長ぴったりに収束
{
    var approx  = new[] { new Vec2(0.1, -0.1), new Vec2(3.2, 0.05), new Vec2(-0.1, 4.1) };
    var lengths = new[] { 3.0, 5.0, 4.0 };   // |P0P1|=3, |P1P2|=5, |P2P0|=4
    var r = PolylineFromLengths.Solve(approx, lengths, closed: true);
    Check("triangle: converged",        r.Converged, $"maxErr={r.MaxError:e3}");
    Check("triangle: edges match",       EdgesMatch(r), string.Join(",", r.EdgeError));
    Check("triangle: no self-intersect", !r.SelfIntersects);
}

// 2) 四角形(閉合): 正方形近傍 + 4辺長 → 辺長一致
{
    var approx  = new[] { new Vec2(0,0), new Vec2(10,0.3), new Vec2(9.8,10), new Vec2(0.2,9.7) };
    var lengths = new[] { 10.0, 10.0, 10.0, 10.0 };
    var r = PolylineFromLengths.Solve(approx, lengths, closed: true);
    Check("square: converged",        r.Converged, $"maxErr={r.MaxError:e3}");
    Check("square: edges match",       EdgesMatch(r), string.Join(",", r.EdgeError));
    Check("square: no self-intersect", !r.SelfIntersects);
}

// 3) 長方形(閉合): 完全一致の入力なら一切動かない(0反復で同一)
{
    var approx  = new[] { new Vec2(0,0), new Vec2(10,0), new Vec2(10,5), new Vec2(0,5) };
    var lengths = new[] { 10.0, 5.0, 10.0, 5.0 };
    var r = PolylineFromLengths.Solve(approx, lengths, closed: true);
    bool unchanged = true;
    for (int i = 0; i < approx.Length; i++) if (Dist(approx[i], r.Points[i]) > 1e-9) unchanged = false;
    Check("rectangle: exact input unchanged", unchanged && r.Converged);
}

// 4) 開放(折れ線): 5点・辺長4本。拘束された3..4辺だけ一致、P[4]->P[0]は自由
{
    var approx  = new[] { new Vec2(0,0), new Vec2(2.1,0.1), new Vec2(4.0,1.5), new Vec2(6.2,1.4), new Vec2(8,3) };
    var lengths = new[] { 2.0, 2.5, 2.0, 3.0 };   // 開放 → n-1 = 4 本
    var r = PolylineFromLengths.Solve(approx, lengths, closed: false);
    Check("open: converged",          r.Converged, $"maxErr={r.MaxError:e3}");
    Check("open: 4 edges match",       EdgesMatch(r), string.Join(",", r.EdgeError));
    Check("open: edge count = n-1",    r.EdgeError.Length == 4);
}

// 5) 重みで頂点固定: P0 を強く固定 → ほとんど動かない
{
    var approx  = new[] { new Vec2(0,0), new Vec2(10,0.5), new Vec2(9.5,10), new Vec2(0.4,9.6) };
    var lengths = new[] { 10.0, 10.0, 10.0, 10.0 };
    var weights = new[] { 1e6, 1.0, 1.0, 1.0 };
    var r = PolylineFromLengths.Solve(approx, lengths, closed: true, weights);
    Check("pinned: P0 barely moves", Dist(approx[0], r.Points[0]) < 1e-3,
          $"moved={Dist(approx[0], r.Points[0]):e3}");
    Check("pinned: edges match",     EdgesMatch(r, 1e-5));
}

// 6) 自己交差検出: 蝶ネクタイ型(頂点順がねじれた四角形)
{
    var bowtie = new[] { new Vec2(0,0), new Vec2(10,10), new Vec2(10,0), new Vec2(0,10) };
    var hits = PolylineFromLengths.FindSelfIntersections(bowtie, closed: true);
    Check("bowtie: detected as self-intersecting", hits.Count > 0, $"crossings={hits.Count}");
}

// 7) 凸四角形は非交差
{
    var convex = new[] { new Vec2(0,0), new Vec2(10,0), new Vec2(10,10), new Vec2(0,10) };
    var hits = PolylineFromLengths.FindSelfIntersections(convex, closed: true);
    Check("convex quad: no self-intersection", hits.Count == 0);
}

// 8) 成立しない辺長(四角形不等式を破る) → 収束しない
{
    var approx  = new[] { new Vec2(0,0), new Vec2(1,0), new Vec2(1,1), new Vec2(0,1) };
    var lengths = new[] { 100.0, 1.0, 1.0, 1.0 };   // 100 > 1+1+1 で閉合不能
    var r = PolylineFromLengths.Solve(approx, lengths, closed: true);
    Check("infeasible lengths: not converged", !r.Converged, $"maxErr={r.MaxError:e3}");
}

// ===== OrthogonalFromLengths(全頂点90°・辺長厳密)tests =====
Console.WriteLine("\nOrthogonalFromLengths tests");

bool LengthsExact(Vec2[] p, double[] L, double tol = 1e-9)
{
    for (int i = 0; i < L.Length; i++) if (Math.Abs(Dist(p[i], p[i + 1]) - L[i]) > tol) return false;
    return true;
}
// 頂点 from..to(両端含む)が直角か
bool RightAngles(Vec2[] p, int from, int to, double tol = 1e-7)
{
    for (int v = from; v <= to; v++)
    {
        double ax = p[v].X - p[v - 1].X, ay = p[v].Y - p[v - 1].Y;
        double bx = p[v + 1].X - p[v].X, by = p[v + 1].Y - p[v].Y;
        double dot = ax * bx + ay * by;
        double sc = Math.Sqrt((ax * ax + ay * ay) * (bx * bx + by * by));
        if (sc < 1e-15 || Math.Abs(dot) / sc > tol) return false;
    }
    return true;
}

var rectApx = new[] { new Vec2(0,0), new Vec2(10,0), new Vec2(10,5), new Vec2(0,5) };
var rectLen = new[] { 10.0, 5.0, 10.0, 5.0 };

// 1) 長方形(閉合): 辺長厳密・全角90°・閉合差ほぼ0
{
    var r = OrthogonalFromLengths.Solve(rectApx, rectLen, closed: true);
    Check("ortho rect: lengths exact", LengthsExact(r.Points, rectLen));
    Check("ortho rect: right angles",  RightAngles(r.Points, 1, rectLen.Length - 1));
    Check("ortho rect: closes",        r.ClosureGap < 1e-7, $"gap={r.ClosureGap:e3}");
    Check("ortho rect: theta ~ 0",     Math.Abs(r.BaseAngleRad) < 1e-9);
}

// 2) 30°回転した長方形: θ≒30°で復元、辺長厳密・全角90°
{
    double t = 30 * Math.PI / 180, c = Math.Cos(t), s = Math.Sin(t);
    var rot = Array.ConvertAll(rectApx, q => new Vec2(c*q.X - s*q.Y, s*q.X + c*q.Y));
    var r = OrthogonalFromLengths.Solve(rot, rectLen, closed: true);
    Check("ortho rot-rect: lengths exact", LengthsExact(r.Points, rectLen));
    Check("ortho rot-rect: right angles",  RightAngles(r.Points, 1, rectLen.Length - 1));
    Check("ortho rot-rect: closes",        r.ClosureGap < 1e-7);
    Check("ortho rot-rect: theta ~ 30deg", Math.Abs(r.BaseAngleRad - t) < 1e-6, $"theta={r.BaseAngleRad:0.######}");
}

// 3) 雑にクリックしても出力は厳密に直角・厳密に辺長
{
    var sloppy = new[] { new Vec2(0,0), new Vec2(9.5,0.8), new Vec2(10.2,5.3), new Vec2(-0.3,4.7) };
    var r = OrthogonalFromLengths.Solve(sloppy, rectLen, closed: true);
    Check("ortho sloppy: right angles",  RightAngles(r.Points, 1, rectLen.Length - 1));
    Check("ortho sloppy: lengths exact", LengthsExact(r.Points, rectLen));
}

// 4) 開放のL字折れ線: 4点・辺長3本。点数=4、内部頂点が直角、辺長厳密
{
    var lApx = new[] { new Vec2(0,0.1), new Vec2(6.0,-0.1), new Vec2(5.9,4.0), new Vec2(2.1,3.9) };
    var lLen = new[] { 6.0, 4.0, 4.0 };           // 開放 → n-1 = 3 本
    var r = OrthogonalFromLengths.Solve(lApx, lLen, closed: false);
    Check("ortho open: point count = n",  r.Points.Length == 4);
    Check("ortho open: lengths exact",    LengthsExact(r.Points, lLen));
    Check("ortho open: right angles",     RightAngles(r.Points, 1, lLen.Length - 1));
}

// 5) 基準辺指定: edge0 を水平(θ=0)に固定
{
    var r = OrthogonalFromLengths.Solve(rectApx, rectLen, closed: true, baseAngleRad: 0.0, alignEdgeIndex: 0);
    Check("ortho ref-angle: edge0 horizontal", Math.Abs(r.Points[1].Y - r.Points[0].Y) < 1e-9);
    Check("ortho ref-angle: lengths exact",    LengthsExact(r.Points, rectLen));
}

Console.WriteLine($"\n{passed} passed, {failed} failed");
return failed == 0 ? 0 : 1;
