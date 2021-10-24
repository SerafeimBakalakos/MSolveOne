using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM.DualMesh;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Tests.MultiphaseThermal.PINN
{
    public class GeometryPreprocessor3D
    {
        private XModel<IXMultiphaseElement> physicalModel;

        public GeometryPreprocessor3D(XModel<IXMultiphaseElement> physicalModel)
        {
            this.physicalModel = physicalModel;
            MinCoords = new double[] { -1.0, -1.0, -1.0 };
            MaxCoords = new double[] { +1.0, +1.0, +1.0 };
        }

        public double[] MinCoords { get; set; }
        public double[] MaxCoords { get; set; }

        public int RngSeed { get; set; } = 33;

        public int[] NumBalls { get; set; } = { 10, 10, 10 };

        public double BallRadius { get; set; } = 20;

        public PhaseGeometryModel GeometryModel { get; set; }

        public string MatrixPhaseName { get; } = "matrix";

        public int MatrixPhaseID { get; set; }

        public string InclusionPhaseName { get; } = "inclusion";

        public List<int> InclusionPhaseIDs { get; set; } = new List<int>();

        public Dictionary<string, double> CalcPhaseVolumes()
        {
            var volumes = new Dictionary<string, double>();
            Dictionary<int, double> phaseVolumes = XFEM.Output.Fields.Utilities.CalcBulkSizeOfEachPhase(physicalModel, GeometryModel);

            volumes[MatrixPhaseName] = phaseVolumes[MatrixPhaseID];

            volumes[InclusionPhaseName] = 0;
            foreach (int phaseID in InclusionPhaseIDs) volumes[InclusionPhaseName] += phaseVolumes[phaseID];

            return volumes;
        }

        public void GeneratePhases(XModel<IXMultiphaseElement> physicalModel, DualCartesianMesh3D mesh)
        {
            GeometryModel = new PhaseGeometryModel(physicalModel);
            physicalModel.GeometryModel = GeometryModel;
            GeometryModel.MergeOverlappingPhases = true;
            var defaultPhase = new DefaultPhase();
            GeometryModel.Phases[defaultPhase.ID] = defaultPhase;
            MatrixPhaseID = 0;

            double margin = 0.0;
            //double margin = 0.01 * (MaxCoordinates[0] - MinCoordinates[0]);
            var minCoordsExtended = new double[2];
            minCoordsExtended[0] = MinCoords[0] - margin;
            minCoordsExtended[1] = MinCoords[1] - margin;
            var maxCoordsExtended = new double[2];
            maxCoordsExtended[0] = MaxCoords[0] + margin;
            maxCoordsExtended[1] = MaxCoords[1] + margin;

            List<Sphere> balls = CreateBalls();
            foreach (Sphere ball in balls)
            {
                // Create phases
                var phaseInclusion = new LsmPhase(GeometryModel.Phases.Count, GeometryModel, -1);
                GeometryModel.Phases[phaseInclusion.ID] = phaseInclusion;
                InclusionPhaseIDs.Add(phaseInclusion.ID);

                // Create phase boundaries
                var lsmInclusion = new GlobalDualMeshLsm3D_OLD(phaseInclusion.ID, mesh, ball);
                //var lsmInclusion = new SimpleLsm3D(phaseInclusion.ID, physicalModel.XNodes, ball);
                var boundary = new ClosedPhaseBoundary(phaseInclusion.ID, lsmInclusion, defaultPhase, phaseInclusion);
                defaultPhase.ExternalBoundaries.Add(boundary);
                defaultPhase.Neighbors.Add(phaseInclusion);
                phaseInclusion.ExternalBoundaries.Add(boundary);
                phaseInclusion.Neighbors.Add(defaultPhase);
            }
        }

        private List<Sphere> CreateBalls()
        {
            double xMin = MinCoords[0], xMax = MaxCoords[0];
            double yMin = MinCoords[1], yMax = MaxCoords[1];
            double zMin = MinCoords[2], zMax = MaxCoords[2];

            var balls = new List<Sphere>(NumBalls[0] * NumBalls[1] * NumBalls[2]);
            double dx = (xMax - xMin) / NumBalls[0];
            double dy = (yMax - yMin) / NumBalls[1];
            double dz = (zMax - yMin) / NumBalls[2];
            int id = 1;
            for (int i = 0; i < NumBalls[0]; ++i)
            {
                double centerX = xMin + dx / 2 + i * dx;
                for (int j = 0; j < NumBalls[1]; ++j)
                {
                    double centerY = yMin + dy / 2 + j * dy;
                    for (int k = 0; k < NumBalls[2]; ++k)
                    {
                        double centerZ = zMin + dz / 2 + k * dz;
                        var sphere = new Sphere(centerX, centerY, centerZ, BallRadius);
                        balls.Add(sphere);
                    }
                }
            }

            return balls;
        }
    }
}
