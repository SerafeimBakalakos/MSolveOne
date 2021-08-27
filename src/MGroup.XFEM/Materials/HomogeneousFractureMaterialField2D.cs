using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Materials.Duplicates;

//TODO: Delete thickness from here
namespace MGroup.XFEM.Materials
{
    public class HomogeneousFractureMaterialField2D : IFractureMaterialField
    {
        private readonly ElasticMaterial2D material;

        public HomogeneousFractureMaterialField2D(double youngModulus, double poissonRatio, double thickness, bool planeStress)
        {
            material = new ElasticMaterial2D(planeStress ? StressState2D.PlaneStress : StressState2D.PlaneStrain);
            material.YoungModulus = youngModulus;
            material.PoissonRatio = poissonRatio;

            this.YoungModulus = youngModulus;
            this.PoissonRatio = poissonRatio;
            if (planeStress)
            {
                this.EquivalentYoungModulus = youngModulus;
                this.EquivalentPoissonRatio = poissonRatio;
                this.Thickness = thickness;
            }
            else
            {
                this.EquivalentYoungModulus = youngModulus / (1.0 - poissonRatio * poissonRatio);
                this.EquivalentPoissonRatio = poissonRatio / (1.0 - poissonRatio);
                this.Thickness = 1;
            }
        }

        public double YoungModulus { get; }
        public double EquivalentYoungModulus { get; }
        public double PoissonRatio { get; }
        public double EquivalentPoissonRatio { get; }
        public double Thickness { get; }

        public IContinuumMaterial FindMaterialAt(XPoint point) => material;
    }
}
