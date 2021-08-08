using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Phases;
using MGroup.XFEM.Output.Fields;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;
using System.Linq;

namespace MGroup.XFEM.Output.Writers
{
	public class PhaseLevelSetPlotter : IPhaseGeometryObserver
	{
		private readonly string directoryPath;
		private readonly PhaseGeometryModel geometryModel;
		private readonly IXModel physicalModel;
		private int iteration;

		public PhaseLevelSetPlotter(string directoryPath, IXModel physicalModel, PhaseGeometryModel geometryModel)
		{
			this.directoryPath = directoryPath.Trim('\\');
			this.physicalModel = physicalModel;
			this.geometryModel = geometryModel;
			iteration = 0;
		}

		public void Update()
		{
			var outMesh = new ContinuousOutputMesh(physicalModel.Nodes.Values.OrderBy(n => n.ID).ToList(), 
				physicalModel.EnumerateElements());
			foreach (IPhaseBoundary boundary in geometryModel.PhaseBoundaries.Values)
			{
				string file = $"{directoryPath}\\level_set{boundary.ID}_t{iteration}.vtk";
				using (var writer = new VtkFileWriter(file))
				{
					var levelSetField = new LevelSetField(physicalModel, boundary.Geometry, outMesh);
					writer.WriteMesh(levelSetField.Mesh);
					writer.WriteScalarField($"inclusion_level_set",
						levelSetField.Mesh, levelSetField.CalcValuesAtVertices());
				}
			}

			++iteration;
		}
	}
}
