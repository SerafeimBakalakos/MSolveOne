using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.XFEM.Materials;

namespace MGroup.XFEM.Cracks.Jintegral
{
    public class HomogeneousSifCalculator : ISifCalculator
    {
        private readonly double equivalentYoungModulus;

        /// <summary>
        /// The material properties (E, v, E*, v*) must be the same across all elements. The user assumes 
        /// responsibility for passing a <see cref="HomogeneousFractureMaterialField2D"/> that has the same properties as 
        /// the materials of all other elements of the integration domain.
        /// </summary>
        /// <param name="globalMaterial">The material properties which must be identical for all elements and this class</param>
        public HomogeneousSifCalculator(HomogeneousFractureMaterialField2D globalMaterial)
        {
            this.equivalentYoungModulus = globalMaterial.EquivalentYoungModulus;
        }

        public double CalculateSif(double interactionIntegral)
        {
            return 0.5 * equivalentYoungModulus * interactionIntegral;
        }
    }
}
