using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.Solvers.DDM.PSM.Vectors;

namespace MGroup.Solvers.DDM.PSM.InterfaceProblem
{
    public class PsmInterfaceProblemVectors
    {
        private readonly IComputeEnvironment environment;
        private readonly IDictionary<int, PsmSubdomainVectors> subdomainVectors;

        public PsmInterfaceProblemVectors(IComputeEnvironment environment, IDictionary<int, PsmSubdomainVectors> subdomainVectors)
        {
            this.environment = environment;
            this.subdomainVectors = subdomainVectors;
        }

        public DistributedOverlappingVector InterfaceProblemRhs { get; private set; }

        public DistributedOverlappingVector InterfaceProblemSolution { get; set; }

        // globalF = sum {Lb[s]^T * (fb[s] - Kbi[s] * inv(Kii[s]) * fi[s]) }
        public void CalcInterfaceRhsVector(DistributedOverlappingIndexer indexer)
        {
            Dictionary<int, Vector> fbCondensed = environment.CreateDictionaryPerNode(
                subdomainID => subdomainVectors[subdomainID].CalcCondensedRhsVector());
            InterfaceProblemRhs = new DistributedOverlappingVector(environment, indexer, fbCondensed);
            InterfaceProblemRhs.SumOverlappingEntries();
        }

        public void Clear()
        {
            InterfaceProblemRhs = null;
        }

    }
}
