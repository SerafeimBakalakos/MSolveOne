using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Extensions;
using MGroup.XFEM.Output.Vtk;

namespace MGroup.XFEM.Output.Writers
{
	public class StructuralBoundaryConditionsPlotter : IModelObserver
	{
		private readonly int dimension;
		private readonly IXModel model;
		private readonly string outputDirectory;
		private int iteration = 0;

		public StructuralBoundaryConditionsPlotter(string outputDirectory, IXModel model)
		{
			this.outputDirectory = outputDirectory;
			this.model = model;
			this.dimension = model.Dimension;
		}

		public void Update()
		{
			if (iteration == 0)
			{
				PlotDirichletBCs();
				PlotNodalLoads();
			}

			++iteration;
		}

		private void PlotDirichletBCs()
		{
			var nodes = new List<double[]>();
			var displacements = new List<double[]>();
			foreach (INode node in model.EnumerateNodes())
			{
				foreach (Constraint constraint in node.Constraints)
				{
					nodes.Add(DrawNode(node));
					displacements.Add(DrawVector(constraint));
				}
			}

			using (var writer = new VtkPointWriter(Path.Combine(outputDirectory, "constrained_nodes.vtk")))
			{
				writer.WritePoints(nodes, true);
				writer.WriteVectorField("prescribed_displacements", displacements);
			}
		}

		private void PlotNodalLoads()
		{
			var nodes = new List<double[]>();
			var loads = new List<double[]>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				foreach (Load load in model.EnumerateNodalLoads(subdomain.ID))
				{
					nodes.Add(DrawNode(load.Node));
					loads.Add(DrawVector(load));
				}
			}
			
			using (var writer = new VtkPointWriter(Path.Combine(outputDirectory, "nodal_loads.vtk")))
			{
				writer.WritePoints(nodes, true);
				writer.WriteVectorField("nodal_loads", loads);
			}
		}

		private double[] DrawNode(INode node)
		{
			if (dimension == 2)
			{
				return new double[] { node.X, node.Y };
			}
			else
			{
				return new double[] { node.X, node.Y, node.Z };
			}
		}

		private double[] DrawVector(Constraint constraint)
		{
			var result = new double[dimension];
			if (constraint.DOF == StructuralDof.TranslationX)
			{
				result[0] = constraint.Amount;
			}
			else if (constraint.DOF == StructuralDof.TranslationY)
			{
				result[1] = constraint.Amount;
			}
			else if (constraint.DOF == StructuralDof.TranslationZ)
			{
				result[2] = constraint.Amount;
			}
			else
			{
				throw new NotImplementedException();
			}
			return result;
		}

		private double[] DrawVector(Load load)
		{
			var result = new double[dimension];
			if (load.DOF == StructuralDof.TranslationX)
			{
				result[0] = load.Amount;
			}
			else if (load.DOF == StructuralDof.TranslationY)
			{
				result[1] = load.Amount;
			}
			else if (load.DOF == StructuralDof.TranslationZ)
			{
				result[2] = load.Amount;
			}
			else
			{
				throw new NotImplementedException();
			}
			return result;
		}
	}
}
