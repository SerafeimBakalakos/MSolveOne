using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Tests.Utilities
{
    public static class Phases
    {
        public static List<ICurve2D> CreateBallsStructured2D(double[] minCoords, double[] maxCoords, int[] numBalls,
            double radius, double distanceFromSidesOverDistanceBetween = 1.0)
        {
            double xMin = minCoords[0], xMax = maxCoords[0], yMin = minCoords[1], yMax = maxCoords[1];
            var curves = new List<ICurve2D>(numBalls[0] * numBalls[1]);
            double dx = (xMax - xMin) / (numBalls[0] - 1 + 2 * distanceFromSidesOverDistanceBetween);
            double dy = (yMax - yMin) / (numBalls[1] - 1 + 2 * distanceFromSidesOverDistanceBetween);
            for (int i = 0; i < numBalls[0]; ++i)
            {
                double centerX = xMin + distanceFromSidesOverDistanceBetween * dx + i * dx;
                for (int j = 0; j < numBalls[1]; ++j)
                {
                    double centerY = yMin + distanceFromSidesOverDistanceBetween * dy + j * dy;
                    var circle = new Circle2D(centerX, centerY, radius);
                    curves.Add(circle);
                }
            }

            return curves;
        }

        public static List<ISurface3D> CreateBallsStructured3D(double[] minCoords, double[] maxCoords, int[] numBalls,
            double radius, double distanceFromSidesOverDistanceBetween = 1.0)
        {
            double xMin = minCoords[0], xMax = maxCoords[0];
            double yMin = minCoords[1], yMax = maxCoords[1];
            double zMin = minCoords[2], zMax = maxCoords[2];

            var balls = new List<ISurface3D>(numBalls[0] * numBalls[1] * numBalls[2]);
            double dx = (xMax - xMin) / (numBalls[0] - 1 + 2 * distanceFromSidesOverDistanceBetween);
            double dy = (yMax - yMin) / (numBalls[1] - 1 + 2 * distanceFromSidesOverDistanceBetween);
            double dz = (zMax - yMin) / (numBalls[2] - 1 + 2 * distanceFromSidesOverDistanceBetween);
            for (int i = 0; i < numBalls[0]; ++i)
            {
                double centerX = xMin + distanceFromSidesOverDistanceBetween * dx + i * dx;
                for (int j = 0; j < numBalls[1]; ++j)
                {
                    double centerY = yMin + distanceFromSidesOverDistanceBetween * dy + j * dy;
                    for (int k = 0; k < numBalls[2]; ++k)
                    {
                        double centerZ = zMin + distanceFromSidesOverDistanceBetween * dz + k * dz;
                        var sphere = new Sphere(centerX, centerY, centerZ, radius);
                        balls.Add(sphere);
                    }
                }
            }

            return balls;
        }

        public static List<ICurve2D> CreateHalfSpace2D(double[] minCoords, double[] maxCoords, bool bottomIsMatrix)
        {
            double[] point0 = { minCoords[0], 0.5 * (minCoords[1] + maxCoords[1])};
            double[] point1 = { maxCoords[0], 0.5 * (minCoords[1] + maxCoords[1]) };
            var curves = new List<ICurve2D>();
            if (bottomIsMatrix)
            {
                var line = new Line2D(point1, point0); // negative towards top, which is the inclusion phase
                curves.Add(line);
            }
            else
            {
                var line = new Line2D(point0, point1); // negative towards bottom, which is the inclusion phase
                curves.Add(line);
            }

            return curves;
        }

        public static List<ISurface3D> CreateHalfSpace3D(double[] minCoords, double[] maxCoords, bool bottomIsMatrix)
        {
            var center = new double[3];
            for (int d = 0; d < 3; ++d) center[d] = 0.5 * (minCoords[d] + maxCoords[d]);
            var surfaces = new List<ISurface3D>();
            double[] normal;
            if (bottomIsMatrix)
            {
                normal = new double[] { 0, -1, 0 }; // positive towards the matrix phase
            }
            else
            {
                normal = new double[] { 0, 1, 0 }; // positive towards the matrix phase
            }
            var plane = Plane3D.CreatePlaneThroughPointWithPositiveNormal(center, normal);
            surfaces.Add(plane);
            return surfaces;
        }

        public static PhaseGeometryModel CreateLsmPhases2D(XModel<IXMultiphaseElement> model, List<ICurve2D> inclusionGeometries,
            bool cartesianMesh = true)
        {
            var geometricModel = new PhaseGeometryModel(model);
            model.GeometryModel = geometricModel;
            var defaultPhase = new DefaultPhase();
            geometricModel.Phases[defaultPhase.ID] = defaultPhase;
            for (int p = 0; p < inclusionGeometries.Count; ++p)
            {
                var lsm = new SimpleLsm2D(p + 1, model.Nodes.Values.ToList(), inclusionGeometries[p], !cartesianMesh);
                var phase = new LsmPhase(p + 1, geometricModel, -1);
                geometricModel.Phases[phase.ID] = phase;

                var boundary = new ClosedPhaseBoundary(phase.ID, lsm, defaultPhase, phase);
                defaultPhase.ExternalBoundaries.Add(boundary);
                defaultPhase.Neighbors.Add(phase);
                phase.ExternalBoundaries.Add(boundary);
                phase.Neighbors.Add(defaultPhase);
                geometricModel.PhaseBoundaries[boundary.ID] = boundary;
            }
            return geometricModel;
        }

        public static PhaseGeometryModel CreateLsmPhases3D(XModel<IXMultiphaseElement> model, 
            List<ISurface3D> inclusionGeometries, bool cartesianMesh = true)
        {
            var geometricModel = new PhaseGeometryModel(model);
            model.GeometryModel = geometricModel;
            var defaultPhase = new DefaultPhase();
            geometricModel.Phases[defaultPhase.ID] = defaultPhase;
            for (int p = 0; p < inclusionGeometries.Count; ++p)
            {
                var lsm = new SimpleLsm3D(p + 1, model.Nodes.Values.ToList(), inclusionGeometries[p], !cartesianMesh);
                var phase = new LsmPhase(p + 1, geometricModel, -1);
                geometricModel.Phases[phase.ID] = phase;

                var boundary = new ClosedPhaseBoundary(phase.ID, lsm, defaultPhase, phase);
                defaultPhase.ExternalBoundaries.Add(boundary);
                defaultPhase.Neighbors.Add(phase);
                phase.ExternalBoundaries.Add(boundary);
                phase.Neighbors.Add(defaultPhase);
                geometricModel.PhaseBoundaries[boundary.ID] = boundary;
            }
            return geometricModel;
        }
    }
}
