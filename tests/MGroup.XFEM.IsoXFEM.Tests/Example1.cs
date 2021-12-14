using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Vectors;
using MGroup.XFEM.Materials.Duplicates;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using System;
using System.Collections.Generic;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution.AlgebraicModel;

namespace MGroup.XFEM.IsoXFEM.Tests
{
   public class Example1
    {
		public enum EndLoad
		{
			UpperEnd,
			MiddleEnd,
			BottomEnd
		}
		private static EndLoad endload;

		public static void RunExample1()
        {
            var geometry = new GeometryProperties(40, 20, 1, 40, 20);
			var material = new ElasticMaterial2D(StressState2D.PlaneStress);
			material.YoungModulus = 1;
			material.PoissonRatio = 0.3;			
			var meshGeneration = new MeshGeneration(material, geometry);
			var mesh=meshGeneration.MakeMesh();
			int dimension = 2;
			var xModel = new XModel<IsoXfemElement2D>(dimension);
			foreach (var item in mesh.Item1.Keys)
			{
				xModel.Nodes[item] = mesh.Item1[item];
			}
			foreach (var item in mesh.Item2.Keys)
			{
				xModel.Elements[item] = mesh.Item2[item];
			}
			xModel.Initialize();
			endload = EndLoad.MiddleEnd;
			int nodeIDLoad =  (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1) - ((int)endload * (geometry.numberOfElementsY)/2) - 1;
			Load load;
			load = new Load()
			{
				Node = xModel.Nodes[nodeIDLoad],
				DOF = StructuralDof.TranslationY,
				Amount = 1
			};
			xModel.NodalLoads.Add(load);
            ISolver solver = new SkylineLdlSolver();
            var femAnalysis = new FEMAnalysis(geometry,xModel, solver/*, rhs*/);
			femAnalysis.Initialize();
            TopologyOptimization.IsoXfem(xModel, femAnalysis);
        }
    }
}
