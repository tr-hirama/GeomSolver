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

Console.WriteLine($"\n{passed} passed, {failed} failed");
return failed == 0 ? 0 : 1;
