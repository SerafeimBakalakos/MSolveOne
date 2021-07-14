using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;

namespace MGroup.Solvers.DDM.Prototypes.FetiDP
{
    public class FetiDPSubdomainStiffnesses
    {
        private readonly IModel model;
        private readonly FetiDPSubdomainDofs dofs;

        public FetiDPSubdomainStiffnesses(IModel model, FetiDPSubdomainDofs dofs)
        {
            this.model = model;
            this.dofs = dofs;
        }

        public BlockMatrix Kcce { get; set; }

        public Dictionary<int, Matrix> Kcc { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Kcre { get; set; }

        public Dictionary<int, Matrix> Kcr { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Krce { get; set; }

        public Dictionary<int, Matrix> Krc { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Krre { get; set; }

        public Dictionary<int, Matrix> Krr { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix InvKrre { get; set; }

        public Dictionary<int, Matrix> InvKrr { get; } = new Dictionary<int, Matrix>();

        public BlockMatrix Scce { get; set; }

        public Dictionary<int, Matrix> Scc { get; } = new Dictionary<int, Matrix>();

        public void CalcAllMatrices(Func<int, IMatrix> getSubdomainKff)
        {
            Kcce = BlockMatrix.Create(dofs.NumSubdomainDofsCorner, dofs.NumSubdomainDofsCorner);
            Kcre = BlockMatrix.Create(dofs.NumSubdomainDofsCorner, dofs.NumSubdomainDofsRemainder);
            Krce = BlockMatrix.Create(dofs.NumSubdomainDofsRemainder, dofs.NumSubdomainDofsCorner);
            Krre = BlockMatrix.Create(dofs.NumSubdomainDofsRemainder, dofs.NumSubdomainDofsRemainder);
            InvKrre = BlockMatrix.Create(dofs.NumSubdomainDofsRemainder, dofs.NumSubdomainDofsRemainder);
            Scce = BlockMatrix.Create(dofs.NumSubdomainDofsCorner, dofs.NumSubdomainDofsCorner);
            foreach (ISubdomain subdomain in model.EnumerateSubdomains())
            {
                int s = subdomain.ID;
                ExtractAndCondense(s, getSubdomainKff(s));
                Kcce.AddBlock(s, s, Kcc[s]);
                Kcre.AddBlock(s, s, Kcr[s]);
                Krce.AddBlock(s, s, Krc[s]);
                Krre.AddBlock(s, s, Krr[s]);
                InvKrre.AddBlock(s, s, InvKrr[s]);
                Scce.AddBlock(s, s, Scc[s]);
            }
        }

        private void ExtractAndCondense(int s, IMatrix Kff)
        {
            int[] cornerToFree = dofs.SubdomainDofsCornerToFree[s];
            int[] remainderToFree = dofs.SubdomainDofsRemainderToFree[s];
            Kcc[s] = (Matrix)Kff.GetSubmatrix(cornerToFree, cornerToFree);
            Kcr[s] = (Matrix)Kff.GetSubmatrix(cornerToFree, remainderToFree);
            Krc[s] = (Matrix)Kff.GetSubmatrix(remainderToFree, cornerToFree);
            Krr[s] = (Matrix)Kff.GetSubmatrix(remainderToFree, remainderToFree);
            InvKrr[s] = Krr[s].Invert();
            Scc[s] = Kcc[s] - Kcr[s] * InvKrr[s] * Krc[s];
        }
    }
}
