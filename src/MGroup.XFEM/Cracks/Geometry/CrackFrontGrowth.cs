﻿using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.Geometry
{
    /// <summary>
    /// Describes the growth of vertices of the crack front. In 2D there are 1-2 vertices (also called tips). In 3D the crack
    /// growth is calculated at various points along the crack front, which is a 3D curve in general.
    /// </summary>
    public class CrackFrontGrowth
    {
        /// <summary>
        /// Angles in the 2D cartesian coordinate system defined at each crack front vertex. Axis x = extension vector. 
        /// Axis y vector normal to crack surface/curve. Units: rad
        /// </summary>
        public double[] AnglesAtFrontVertices { get; set; }

        /// <summary>
        /// The length of the growth at each crack front vertex, along the direction indicated by the corresponding entry of 
        /// <see cref="AnglesAtFrontVertices"/>.
        /// </summary>
        public double[] LengthsAtFrontVertices { get; set; }
    }
}
