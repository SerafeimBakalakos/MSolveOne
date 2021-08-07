using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

//TODO: Remove duplication between Line and LineSegment.
namespace MGroup.XFEM.Geometry.Primitives
{
    public class LineSegment2D : Line2D
    {
        private LineSegment2D(double[] start, double[] end, double length, double cosa, double sina, double[] originLocal)
            : base(cosa, sina, originLocal)
        {
            this.Start = start;
            this.End = end;
            this.Length = length;
        }

        public double[] End { get; }

        public double Length { get; }

        public double[] Start { get; }

        public static LineSegment2D Create(double[] start, double[] end)
        {
            double dx = end[0] - start[0];
            double dy = end[1] - start[1];

            double length = Math.Sqrt(dx * dx + dy * dy);
            double cosa = dx / length;
            double sina = dy / length;

            var originLocal = new double[2];
            originLocal[0] = -cosa * start[0] - sina * start[1];
            originLocal[1] = sina * start[0] - cosa * start[1];

            return new LineSegment2D(start, end, length, cosa, sina, originLocal);
        }

        public IElementDiscontinuityInteraction IntersectPolygon(IList<double[]> nodes)
        {
            //TODO: needs a fast way to eliminate most elements

            IElementDiscontinuityInteraction lineIntersection = base.IntersectPolygon(nodes);
            if (lineIntersection is LineSegmentIntersection2D segment)
            {
                // The intersection points may not be inside the segment. 
                // Find the actual start and end of the intersection segment.
                double startLocalX = Math.Max(0, segment.StartLocalX);
                double endLocalX = Math.Min(Length, segment.EndLocalX);

                if (startLocalX >= endLocalX) return new NullElementDiscontinuityInteraction(-1, null);
                else
                {
                    return new LineSegmentIntersection2D(segment.RelativePosition, cosa, sina, originLocal, 
                        startLocalX, endLocalX);
                }
            }
            else return lineIntersection;
        }

        public double[] TransformGlobalToLocalPoint(double[] point)
        {
            return new double[]
            {
                cosa * point[0] + sina * point[1] + originLocal[0],
                -sina * point[0] + cosa * point[1] + originLocal[1]
            };
        }
    }
}
