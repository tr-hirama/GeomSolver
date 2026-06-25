using System;
using System.Collections.Generic;

namespace Geometry
{
    /// <summary>
    /// 概略の頂点位置と各辺の指定長から、全頂点が厳密に90°(直交多角形/折れ線)で
    /// 各辺長も厳密に保たれる図形を作る。基準角θは概略点から最小二乗で推定するか、
    /// baseAngleRad で指定(alignEdgeIndex の辺をその方向に合わせる)。
    /// 閉合は強制しない(過剰拘束のため)。closed=true のときは閉合差 ClosureGap を返す。
    ///
    /// 出力 Points は直交折れ線 P0..Pm (m=辺数)。
    /// closed の場合 Points.Length = n+1 で末尾 Pm は始点への戻り点(Pm≒P0 なら閉じている)。
    /// open の場合 Points.Length = n。
    /// </summary>
    public static class OrthogonalFromLengths
    {
        public sealed class Result
        {
            public Vec2[] Points       = Array.Empty<Vec2>();
            public double BaseAngleRad;                       // 基準角 θ(ラジアン)
            public double ClosureGap;                         // closed: ||Pm - P0|| / open: 0
            public double FitResidual;                        // 概略点へのRMS残差
            public int[]  TurnSigns    = Array.Empty<int>();  // 各内部頂点の回転 +1=左90° / -1=右90°
            public bool   HasAmbiguousCorner;                 // 概略点が直角向きを判定しにくい箇所あり
        }

        /// <param name="approx">概略頂点(クリック点)。</param>
        /// <param name="lengths">辺長(厳密に保たれる)。閉合なら n 個、開放なら n-1 個。</param>
        /// <param name="closed">閉合差を測るか(true)。形状は常に開チェーンとして配置。</param>
        /// <param name="baseAngleRad">基準角を直接指定(null なら概略点から推定)。</param>
        /// <param name="alignEdgeIndex">baseAngleRad 指定時、その角度に合わせる辺の番号。</param>
        public static Result Solve(
            IReadOnlyList<Vec2> approx,
            IReadOnlyList<double> lengths,
            bool closed,
            double? baseAngleRad = null,
            int alignEdgeIndex = 0)
        {
            int n = approx.Count;
            if (n < 2) throw new ArgumentException("頂点は2点以上必要");
            int m = closed ? n : n - 1;                // 辺数
            if (lengths.Count != m)
                throw new ArgumentException($"辺長は{(closed ? "閉合" : "開放")}で {m} 個必要");

            // 1) 概略点から各辺の回転(±90°)を決める。k[i] = 90°単位の累積方向。
            var k = new int[m];
            var turns = new int[Math.Max(0, m - 1)];
            bool ambiguous = false;
            k[0] = 0;
            for (int i = 1; i < m; i++)
            {
                Vec2 p0 = approx[i - 1], p1 = approx[i], p2 = approx[(i + 1) % n];
                double ax = p1.X - p0.X, ay = p1.Y - p0.Y;
                double bx = p2.X - p1.X, by = p2.Y - p1.Y;
                double cross = ax * by - ay * bx;
                double scale = Math.Sqrt((ax*ax + ay*ay) * (bx*bx + by*by));
                int s = cross >= 0 ? 1 : -1;           // 左折=+90° / 右折=-90°
                if (scale <= 1e-12 || Math.Abs(cross) < 1e-6 * scale) ambiguous = true;
                turns[i - 1] = s;
                k[i] = k[i - 1] + s;
            }

            // 2) θ=0 のテンプレート頂点 a[0..m] (辺長厳密・全角90°で構成)
            var a = new Vec2[m + 1];
            a[0] = new Vec2(0, 0);
            for (int i = 0; i < m; i++)
            {
                var u = Unit90(k[i]);
                a[i + 1] = new Vec2(a[i].X + lengths[i] * u.X, a[i].Y + lengths[i] * u.Y);
            }

            int fitN = n;                              // 概略点に対応させる頂点数(先頭 n 個)

            // 3) 基準角 θ
            double theta;
            if (baseAngleRad.HasValue)
            {
                int ai = Math.Clamp(alignEdgeIndex, 0, m - 1);
                theta = baseAngleRad.Value - k[ai] * (Math.PI / 2.0);
            }
            else
            {
                // 2D プロクラステス: a[0..fitN-1] を approx に最小二乗で回転合わせ
                double cax = 0, cay = 0, cqx = 0, cqy = 0;
                for (int i = 0; i < fitN; i++) { cax += a[i].X; cay += a[i].Y; cqx += approx[i].X; cqy += approx[i].Y; }
                cax /= fitN; cay /= fitN; cqx /= fitN; cqy /= fitN;
                double sDot = 0, sCross = 0;
                for (int i = 0; i < fitN; i++)
                {
                    double ux = a[i].X - cax, uy = a[i].Y - cay;
                    double wx = approx[i].X - cqx, wy = approx[i].Y - cqy;
                    sDot   += ux * wx + uy * wy;
                    sCross += ux * wy - uy * wx;
                }
                theta = (sDot == 0 && sCross == 0) ? 0.0 : Math.Atan2(sCross, sDot);
            }

            // 4) 平行移動(最小二乗): P0 = Q̄ - R(θ)·ā、そして全点配置
            double cos = Math.Cos(theta), sin = Math.Sin(theta);
            double abx = 0, aby = 0, qbx = 0, qby = 0;
            for (int i = 0; i < fitN; i++) { abx += a[i].X; aby += a[i].Y; qbx += approx[i].X; qby += approx[i].Y; }
            abx /= fitN; aby /= fitN; qbx /= fitN; qby /= fitN;
            double rabx = cos * abx - sin * aby, raby = sin * abx + cos * aby;
            double p0x = qbx - rabx, p0y = qby - raby;

            var pts = new Vec2[m + 1];
            for (int i = 0; i <= m; i++)
            {
                double rx = cos * a[i].X - sin * a[i].Y;
                double ry = sin * a[i].X + cos * a[i].Y;
                pts[i] = new Vec2(p0x + rx, p0y + ry);
            }

            // 5) 残差・閉合差
            double sse = 0;
            for (int i = 0; i < fitN; i++)
            {
                double dx = pts[i].X - approx[i].X, dy = pts[i].Y - approx[i].Y;
                sse += dx*dx + dy*dy;
            }
            double resid = Math.Sqrt(sse / fitN);

            double gap = 0;
            if (closed)
            {
                double dx = pts[m].X - pts[0].X, dy = pts[m].Y - pts[0].Y;
                gap = Math.Sqrt(dx*dx + dy*dy);
            }

            // Points: open は P0..P(n-1) の n 点、closed は P0..Pn の n+1 点(末尾=戻り点)
            return new Result {
                Points = pts, BaseAngleRad = theta, ClosureGap = gap,
                FitResidual = resid, TurnSigns = turns, HasAmbiguousCorner = ambiguous
            };
        }

        private static Vec2 Unit90(int k)
        {
            int r = ((k % 4) + 4) % 4;       // 0:+x 1:+y 2:-x 3:-y
            return r switch
            {
                0 => new Vec2(1, 0),
                1 => new Vec2(0, 1),
                2 => new Vec2(-1, 0),
                _ => new Vec2(0, -1),
            };
        }
    }
}
