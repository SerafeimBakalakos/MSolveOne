using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials.Duplicates;

namespace MGroup.XFEM.Materials
{
    public class HomogeneousFractureMaterialField3D : IFractureMaterialField
    {
        private readonly ElasticMaterial3D material;

        public HomogeneousFractureMaterialField3D(double youngModulus, double poissonRatio)
        {
            this.material = new ElasticMaterial3D();
            this.material.YoungModulus = youngModulus;
            this.material.PoissonRatio = poissonRatio;

            this.YoungModulus = youngModulus;
            this.PoissonRatio = poissonRatio;
            this.EquivalentYoungModulus = youngModulus;
            this.EquivalentPoissonRatio = poissonRatio;
        }

        public double YoungModulus { get; }
        public double EquivalentYoungModulus { get; }
        public double PoissonRatio { get; }
        public double EquivalentPoissonRatio { get; }

        public IContinuumMaterial FindMaterialAt(XPoint point) => material;
    }
}
