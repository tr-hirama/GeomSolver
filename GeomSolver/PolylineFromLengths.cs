using System;
using System.Collections.Generic;

namespace Geometry
{
    /// <summary>2次元座標。</summary>
    public readonly struct Vec2
    {
        public readonly double X, Y;
        public Vec2(double x, double y) { X = x; Y = y; }
        public override string ToString() => $"({X:0.###},{Y:0.###})";
    }

    /// <summary>
    /// 概略の頂点位置と各辺の指定長から、辺長拘束をちょうど満たし
    /// かつ概略位置からの移動量が最小になる多角形/折れ線を求める。
    /// closed=true なら最終辺 P[n-1]->P[0] も拘束に含める(閉合)。
    /// </summary>
    public static class PolylineFromLengths
    {
        public sealed class Result
        {
            public Vec2[]   Points     = Array.Empty<Vec2>();
            public double[] EdgeError  = Array.Empty<double>(); // |実長 - 指定長|
            public double   MaxError;                            // 閉合差/最大辺誤差
            public int      Iterations;
            public bool     Converged;
            public bool     SelfIntersects;                      // 自己交差あり
            public List<(int EdgeA, int EdgeB)> Crossings = new();// 交差した辺の組
        }

        /// <param name="approx">概略頂点(クリック点)。</param>
        /// <param name="lengths">辺長。閉合なら n 個、開放なら n-1 個。</param>
        /// <param name="closed">閉合する/しない。</param>
        /// <param name="weights">頂点ごとの重み(大きいほど動かない=固定)。既定 1。</param>
        public static Result Solve(
            IReadOnlyList<Vec2> approx,
            IReadOnlyList<double> lengths,
            bool closed,
            IReadOnlyList<double>? weights = null,
            double tol = 1e-7,
            int maxIter = 64)
        {
            int n = approx.Count;
            if (n < 2) throw new ArgumentException("頂点は2点以上必要");
            int m = closed ? n : n - 1;                 // 拘束(辺)の数
            if (lengths.Count != m)
                throw new ArgumentException($"辺長は{(closed ? "閉合" : "開放")}で {m} 個必要");

            int nv = 2 * n;                              // 変数 [x0,y0,x1,y1,...]
            var x = new double[nv];
            var q = new double[nv];
            for (int i = 0; i < n; i++)
            {
                x[2*i]   = q[2*i]   = approx[i].X;
                x[2*i+1] = q[2*i+1] = approx[i].Y;
            }
            var w = new double[n];
            for (int i = 0; i < n; i++) w[i] = weights != null ? weights[i] : 1.0;

            int dim = nv + m;                            // KKT系サイズ
            var A = new double[dim, dim];
            var b = new double[dim];

            int iter = 0;
            for (; iter < maxIter; iter++)
            {
                Array.Clear(A, 0, A.Length);
                Array.Clear(b, 0, b.Length);

                // (1,1)ブロック W と 右辺 -∇f = -W(x-q)
                for (int i = 0; i < n; i++)
                {
                    A[2*i,   2*i]   = w[i];
                    A[2*i+1, 2*i+1] = w[i];
                    b[2*i]   = -w[i] * (x[2*i]   - q[2*i]);
                    b[2*i+1] = -w[i] * (x[2*i+1] - q[2*i+1]);
                }

                double maxC = 0;
                for (int k = 0; k < m; k++)
                {
                    int a = k, bb = (k + 1) % n;         // closedで最後は0へ
                    double dx = x[2*bb]   - x[2*a];
                    double dy = x[2*bb+1] - x[2*a+1];
                    double c  = dx*dx + dy*dy - lengths[k]*lengths[k];

                    int row = nv + k;
                    A[row, 2*a]   = -2*dx; A[2*a,   row] = -2*dx;
                    A[row, 2*a+1] = -2*dy; A[2*a+1, row] = -2*dy;
                    A[row, 2*bb]  =  2*dx; A[2*bb,  row] =  2*dx;
                    A[row, 2*bb+1]=  2*dy; A[2*bb+1,row] =  2*dy;
                    b[row] = -c;
                    maxC = Math.Max(maxC, Math.Abs(c));
                }

                if (iter > 0 && Math.Sqrt(maxC) < tol) break;

                var sol = SolveLinear(A, b, dim);
                if (sol == null) break;                  // 特異(退化幾何)

                double step = 0;
                for (int i = 0; i < nv; i++) { x[i] += sol[i]; step = Math.Max(step, Math.Abs(sol[i])); }
                if (step < tol) { iter++; break; }
            }

            var pts = new Vec2[n];
            for (int i = 0; i < n; i++) pts[i] = new Vec2(x[2*i], x[2*i+1]);
            var err = new double[m];
            double maxErr = 0;
            for (int k = 0; k < m; k++)
            {
                int a = k, bb = (k + 1) % n;
                double dx = pts[bb].X - pts[a].X, dy = pts[bb].Y - pts[a].Y;
                err[k] = Math.Abs(Math.Sqrt(dx*dx + dy*dy) - lengths[k]);
                maxErr = Math.Max(maxErr, err[k]);
            }

            var res = new Result {
                Points = pts, EdgeError = err, MaxError = maxErr,
                Iterations = iter, Converged = maxErr < 1e-4
            };
            res.Crossings      = FindSelfIntersections(pts, closed);
            res.SelfIntersects = res.Crossings.Count > 0;
            return res;
        }

        /// <summary>
        /// 自己交差している辺の組(インデックス)を列挙する。
        /// 隣接辺(端点共有)・閉合の先頭末尾辺は除外。closed=false は折れ線扱い。
        /// </summary>
        public static List<(int A, int B)> FindSelfIntersections(
            IReadOnlyList<Vec2> pts, bool closed, double eps = 1e-9)
        {
            int n = pts.Count;
            int m = closed ? n : n - 1;              // 辺数
            var hits = new List<(int, int)>();
            for (int i = 0; i < m; i++)
            {
                Vec2 a1 = pts[i], a2 = pts[(i + 1) % n];
                for (int j = i + 1; j < m; j++)
                {
                    if (j == i + 1) continue;                         // 隣接辺
                    if (closed && i == 0 && j == m - 1) continue;     // 閉合の先頭⇔末尾
                    Vec2 b1 = pts[j], b2 = pts[(j + 1) % n];
                    if (SegmentsIntersect(a1, a2, b1, b2, eps)) hits.Add((i, j));
                }
            }
            return hits;
        }

        private static double Cross(Vec2 a, Vec2 b, Vec2 c)   // (b-a)×(c-a)
            => (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);

        private static int Sign(double v, double eps) => v > eps ? 1 : (v < -eps ? -1 : 0);

        private static bool OnSeg(Vec2 a, Vec2 b, Vec2 p, double eps)  // p が線分ab上(共線前提)
            => Math.Min(a.X, b.X) - eps <= p.X && p.X <= Math.Max(a.X, b.X) + eps
            && Math.Min(a.Y, b.Y) - eps <= p.Y && p.Y <= Math.Max(a.Y, b.Y) + eps;

        private static bool SegmentsIntersect(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4, double eps)
        {
            int d1 = Sign(Cross(p3, p4, p1), eps);
            int d2 = Sign(Cross(p3, p4, p2), eps);
            int d3 = Sign(Cross(p1, p2, p3), eps);
            int d4 = Sign(Cross(p1, p2, p4), eps);
            if (d1 != d2 && d3 != d4) return true;                 // 真に交差
            if (d1 == 0 && OnSeg(p3, p4, p1, eps)) return true;    // 端点接触/共線重なり
            if (d2 == 0 && OnSeg(p3, p4, p2, eps)) return true;
            if (d3 == 0 && OnSeg(p1, p2, p3, eps)) return true;
            if (d4 == 0 && OnSeg(p1, p2, p4, eps)) return true;
            return false;
        }

        // 部分ピボット付きガウス消去で A x = b を解く(特異なら null)
        private static double[]? SolveLinear(double[,] A, double[] b, int dim)
        {
            var M = (double[,])A.Clone();
            var v = (double[])b.Clone();
            for (int col = 0; col < dim; col++)
            {
                int piv = col; double best = Math.Abs(M[col, col]);
                for (int r = col + 1; r < dim; r++)
                {
                    double a = Math.Abs(M[r, col]);
                    if (a > best) { best = a; piv = r; }
                }
                if (best < 1e-14) return null;            // 特異
                if (piv != col)
                {
                    for (int j = 0; j < dim; j++) { var t = M[col, j]; M[col, j] = M[piv, j]; M[piv, j] = t; }
                    (v[col], v[piv]) = (v[piv], v[col]);
                }
                double diag = M[col, col];
                for (int r = 0; r < dim; r++)
                {
                    if (r == col) continue;
                    double f = M[r, col] / diag;
                    if (f == 0) continue;
                    for (int j = col; j < dim; j++) M[r, j] -= f * M[col, j];
                    v[r] -= f * v[col];
                }
            }
            var x = new double[dim];
            for (int i = 0; i < dim; i++) x[i] = v[i] / M[i, i];
            return x;
        }
    }
}
