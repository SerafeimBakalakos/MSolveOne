using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{	
	public class MaterialProperties
    {
        /// <summary>
        /// The Young Modulus of the material.
        /// </summary>
        public double YoungModulus { get; }
        /// <summary>
        /// The Poisson Ratio of the material.
        /// </summary>
        public double PoissonRatio { get; }

        /// <summary>
        /// Constructs a Material object and initializes it with its young modulus and poisson ratio.
        /// </summary>
        /// <param name="youngModulus"> The young modulus of the material.</param>
        /// <param name="poissonRatio"> The poisson ratio of the material.</param>
        public MaterialProperties(double youngModulus, double poissonRatio)
        {
            YoungModulus = youngModulus;
            PoissonRatio = poissonRatio;
        }
    }
}
