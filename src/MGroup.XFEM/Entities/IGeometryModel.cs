using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Distributed;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.XFEM.Enrichment.Enrichers;

namespace MGroup.XFEM.Entities
{
    public interface IGeometryModel
    {
        INodeEnricher Enricher { get; }

        IXDiscontinuity GetDiscontinuity(int discontinuityID);

        IEnumerable<IXDiscontinuity> EnumerateDiscontinuities();

        void InitializeGeometry();

        void InteractWithMesh();

        void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector solutionFreeDofs);
    }
}
