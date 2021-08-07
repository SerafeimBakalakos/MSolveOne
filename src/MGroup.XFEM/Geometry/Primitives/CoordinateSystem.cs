using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Geometry.Primitives
{
    public enum CoordinateSystem
    {
        GlobalCartesian, ElementNatural, IntegrationSubcellLocal, CurveLocal, CrackTipCartesian, 
        
        /// <summary>
        /// (r, theta), where r = distance from origin, theta = anti-clockwise angle from axis +x.
        /// </summary>
        CrackTipPolar
    }
}
