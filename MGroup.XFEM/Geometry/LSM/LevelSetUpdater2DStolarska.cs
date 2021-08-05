using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Geometry.LSM
{
    public class LevelSetUpdater2DStolarska : IOpenLevelSetUpdater
    {
        public LevelSetUpdater2DStolarska() { }

        public void Update(double[] oldTip, double[] newTip, IEnumerable<XNode> nodes,
            Dictionary<int, double> levelSetsBody, Dictionary<int, double> levelSetsTip)
        {
            double dx = newTip[0] - oldTip[0];
            double dy = newTip[1] - oldTip[1];
            double length = Math.Sqrt(dx * dx + dy * dy);
            double unitDx = dx / length;
            double unitDy = dy / length;
            var newSegment = LineSegment2D.Create(oldTip, newTip);

            foreach (XNode node in nodes)
            {
                // Rotate the ALL tip level sets towards the new tip and then advance them
                double rotatedTipLevelSet = (node.Coordinates[0] - oldTip[0]) * unitDx 
                    + (node.Coordinates[1] - oldTip[1]) * unitDy;
                levelSetsTip[node.ID] = rotatedTipLevelSet - length;

                // Only some body level sets are updated (See Stolarska 2001) 
                // WARNING: this is quite dangerous. If the crack turns/kinks sufficiently over its whole history, then nodes far
                // alway from the tip that should remain unchanged, will get new incorrect level sets. A new formula is needed to 
                // avoid messing them up. This may not be a problem for LSM using higher order elements.
                // TODO: In a narrow band approach, it may be possible to limit the level set update in nodes
                // near to the old tip the new tip or the inbetween segment.
                if (rotatedTipLevelSet > 0.0) 
                {
                    levelSetsBody[node.ID] = newSegment.SignedDistanceOf(node.Coordinates);
                }
            }
        }
    }
}
