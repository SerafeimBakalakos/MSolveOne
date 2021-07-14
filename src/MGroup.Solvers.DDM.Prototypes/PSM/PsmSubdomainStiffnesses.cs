using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;

namespace MGroup.Solvers.DDM.Prototypes.PSM
{
    public class PsmSubdomainStiffnesses
    {
        private readonly IModel model;
        private readonly PsmSubdomainDofs dofs;

        public PsmSubdomainStiffnesses(IModel model, PsmSubdomainDofs dofs)
        {
            this.model = model;
            this.dofs = dofs;
        }

        public BlockMatrix Kbbe { get; set; }

        public Dictionary<int, Matrix> Kbb { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Kbie { get; set; }

        public Dictionary<int, Matrix> Kbi { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Kibe { get; set; }

        public Dictionary<int, Matrix> Kib { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Kiie { get; set; }

        public Dictionary<int, Matrix> Kii { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix InvKiie { get; set; }

        public Dictionary<int, Matrix> InvKii { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Sbbe { get; set; }

        public Dictionary<int, Matrix> Sbb { get; } = new Dictionary<int, Matrix>();

        public void CalcAllMatrices(Func<int, IMatrix> getSubdomainKff)
        {
            Kbbe = BlockMatrix.Create(dofs.NumSubdomainDofsBoundary, dofs.NumSubdomainDofsBoundary);
            Kbie = BlockMatrix.Create(dofs.NumSubdomainDofsBoundary, dofs.NumSubdomainDofsInternal);
            Kibe = BlockMatrix.Create(dofs.NumSubdomainDofsInternal, dofs.NumSubdomainDofsBoundary);
            Kiie = BlockMatrix.Create(dofs.NumSubdomainDofsInternal, dofs.NumSubdomainDofsInternal);
            InvKiie = BlockMatrix.Create(dofs.NumSubdomainDofsInternal, dofs.NumSubdomainDofsInternal);
            Sbbe = BlockMatrix.Create(dofs.NumSubdomainDofsBoundary, dofs.NumSubdomainDofsBoundary);
            foreach (ISubdomain subdomain in model.EnumerateSubdomains())
            {
                int s = subdomain.ID;
                ExtractAndCondense(s, getSubdomainKff(s));
                Kbbe.AddBlock(s, s, Kbb[s]);
                Kbie.AddBlock(s, s, Kbi[s]);
                Kibe.AddBlock(s, s, Kib[s]);
                Kiie.AddBlock(s, s, Kii[s]);
                InvKiie.AddBlock(s, s, InvKii[s]);
                Sbbe.AddBlock(s, s, Sbb[s]);
            }
        }

        private void ExtractAndCondense(int s, IMatrix Kff)
        {
            int[] boundaryToFree = dofs.SubdomainDofsBoundaryToFree[s];
            int[] internalToFree = dofs.SubdomainDofsInternalToFree[s];
            Kbb[s] = (Matrix)Kff.GetSubmatrix(boundaryToFree, boundaryToFree);
            Kbi[s] = (Matrix)Kff.GetSubmatrix(boundaryToFree, internalToFree);
            Kib[s] = (Matrix)Kff.GetSubmatrix(internalToFree, boundaryToFree);
            Kii[s] = (Matrix)Kff.GetSubmatrix(internalToFree, internalToFree);
            InvKii[s] = Kii[s].Invert();
            Sbb[s] = Kbb[s] - Kbi[s] * InvKii[s] * Kib[s];
        }
    }
}
