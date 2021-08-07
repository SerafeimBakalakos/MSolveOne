using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Output.Writers
{
    public class CrackLevelSetPlotter : ICrackObserver
    {
        private readonly ExteriorLsmCrack crack;
        private readonly ContinuousOutputMesh mesh;
        private readonly string outputDirectory;
        private int iteration;

        public CrackLevelSetPlotter(ExteriorLsmCrack crack, ContinuousOutputMesh mesh, string outputDirectory)
        {
            this.crack = crack;
            this.mesh = mesh;
            this.outputDirectory = outputDirectory;
            iteration = 0;
        }

        public void Update()
        {
            // Log the crack path
            using (var crackWriter = new VtkPolylineWriter($"{outputDirectory}\\crack_path_{iteration}.vtk"))
            {
                crackWriter.WritePolyline(crack.CrackPath);
            }

            // Log the level sets
            using (var lsmWriter = new VtkFileWriter($"{outputDirectory}\\level_sets_{iteration}.vtk"))
            {
                // The mesh should be provided by a dedicated IVtkMesh. Its implementations will be continuous/discontinuous, 
                // FEM/XFEM. These classes should then replace the relevant code in DiscontinuousMeshVTKWriter.cs and Logging project.
                // Also the Dictionary<Element, VtkCell> should be avoided. Just store the VtkCells in the same order as the IList<Element> of the model.
                // Ideally VtkPoint can be completely replaced by INode and perhaps VtkCell can be replaced by IElement or ICell<TNode>.
                lsmWriter.WriteMesh(mesh);

                var levelSetsBody = new double[mesh.NumOutVertices];
                var levelSetsTip = new double[mesh.NumOutVertices];
                int i = 0;
                foreach (XNode node in mesh.OriginalVertices)
                {
                    levelSetsBody[i] = crack.LsmGeometry.LevelSets[node.ID];
                    levelSetsTip[i] = crack.LsmGeometry.LevelSetsTip[node.ID];
                    ++i;
                }

                lsmWriter.WriteScalarField("level_set_crack_body", mesh, levelSetsBody);
                lsmWriter.WriteScalarField("level_set_crack_tip", mesh, levelSetsTip);
            }
            ++iteration;
        }
    }
}
