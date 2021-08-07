using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;

namespace MGroup.XFEM.Materials
{
    public class CohesiveInterfaceMaterial
    {
        public CohesiveInterfaceMaterial(IMatrix interfaceConductivity)
        {
            this.ConstitutiveMatrix = interfaceConductivity;
        }

        /// <summary>
        /// In 2D: [ dtn/d[un], dtn/d[us] ; dts/d[un], dts/d[us] ], where s is the tangential direction to the interface and 
        /// n the normal direction.
        /// </summary>
        public IMatrix ConstitutiveMatrix { get; }

        public CohesiveInterfaceMaterial Clone() => new CohesiveInterfaceMaterial(ConstitutiveMatrix.Copy());
    }
}
