using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Cracks.PropagationTermination
{
    public class FractureToughnessTermination : IPropagationTermination
    {
        private readonly double fractureToughness;
        private readonly IEquivalentSifCalculator equivalentSifCalculator;

        private bool mustTerminate;

        public FractureToughnessTermination(double fractureToughness, IEquivalentSifCalculator equivalentSifCalculator)
        {
            this.fractureToughness = fractureToughness;
            this.equivalentSifCalculator = equivalentSifCalculator;
        }

        public FractureToughnessTermination(double fractureToughness)
            : this(fractureToughness, new EquivalentSifCalculatorNorm())
        {
        }

        public string Description { get; } = "Stress intensity factor exceeded material's fracture toughness." 
            + " This leads to the crack propagating uncontrollably throughout the whole domain.";

        public bool MustTerminate(ICrack crack)
        {
            crack.CheckPropagation(this);
            return mustTerminate;
        }

        public void Update(double[] sifs, double[] newCrackTip)
        {
            mustTerminate = false;
            double equivalentSif = equivalentSifCalculator.CalculateEquivalentSIF(sifs);
            if (equivalentSif >= fractureToughness) mustTerminate = true;
            else mustTerminate = false;
        }
    }
}
