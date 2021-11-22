using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{
    public enum EndLoad
    {
        UpperEnd,
        MiddleEnd,
        BottomEnd
    }
    class NodalLoad
    {
        
        public GeometryProperties geometry;
        private readonly EndLoad endload;
        public double MagnitudeForce { get; set; } = 1;                
        public NodalLoad(GeometryProperties geometry, EndLoad endload=EndLoad.MiddleEnd)
        {
            this.geometry = geometry;
            this.endload = endload;
        }
        public Vector CalcRHS()
        {
            int dofLoad = 2 * (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * geometry.numberOfElementsY) - 1;
            Vector load = Vector.CreateZero(2 * (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1));
            load[dofLoad] = MagnitudeForce;
            return load;
        }
    }
}
