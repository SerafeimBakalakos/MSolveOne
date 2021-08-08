using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;
using Troschuetz.Random.Distributions.Continuous;

namespace MGroup.XFEM.Tests.MultiphaseThermal.EpoxyAg
{
	public class GeometryPreprocessor3DRandomThickness : GeometryPreprocessorBase
	{
		public GeometryPreprocessor3DRandomThickness(XModel<IXMultiphaseElement> physicalModel) : base(physicalModel)
		{
			MinCoordinates = new double[] { -1.0, -1.0, -1.0 };
			MaxCoordinates = new double[] { +1.0, +1.0, +1.0 };
		}

		public void GeneratePhases(XModel<IXMultiphaseElement> physicalModel)
		{
			GeometryModel = new PhaseGeometryModel(physicalModel);
			physicalModel.GeometryModel = GeometryModel;
			GeometryModel.Enricher = NodeEnricherMultiphaseNoJunctions.CreateThermalStep(GeometryModel);
			GeometryModel.MergeOverlappingPhases = true;
			var defaultPhase = new DefaultPhase();
			GeometryModel.Phases[defaultPhase.ID] = defaultPhase;
			MatrixPhaseID = 0;

			double margin = 0.0;
			//double margin = 0.01 * (MaxCoordinates[0] - MinCoordinates[0]);
			var minCoordsExtended = new double[2];
			minCoordsExtended[0] = MinCoordinates[0] - margin;
			minCoordsExtended[1] = MinCoordinates[1] - margin;
			var maxCoordsExtended = new double[2];
			maxCoordsExtended[0] = MaxCoordinates[0] + margin;
			maxCoordsExtended[1] = MaxCoordinates[1] + margin;

			var ballsInternal = new List<Sphere>();
			var ballsExternal = new List<Sphere>();

			var radiiOfEpoxyDistributionNormal = new NormalDistribution(27644437, 0, 1);
			double densitySilver = 10.49;
			double densityEpoxy = 1.2;
			double weightFraction = 0.3;
			//double weightFraction = 0.08;
			int b = 0;
			var rng = new Random(RngSeed);
			while (b < NumBalls)
			{
				var newCenter = new double[3];
				newCenter[0] = rng.NextDouble() * (maxCoordsExtended[0] - minCoordsExtended[0]) + minCoordsExtended[0];
				newCenter[1] = rng.NextDouble() * (maxCoordsExtended[1] - minCoordsExtended[1]) + minCoordsExtended[1];
				newCenter[2] = rng.NextDouble() * (maxCoordsExtended[2] - minCoordsExtended[2]) + minCoordsExtended[2];

				double radiusEpoxyPhase = 0.5 * Math.Exp(6.6032 + 0.2462 * radiiOfEpoxyDistributionNormal.NextDouble());

				// This is the correct one
				//double radiusExternal = Math.Pow(1 + densityEpoxy / densitySilver * weightFraction, 1.0 / 3) * radiusEpoxyPhase;
				// Hack to test code
				#region debug
				//double radiusExternal = Math.Pow(1 + densityEpoxy / densitySilver * weightFraction, 1.0 / 3) * radiusEpoxyPhase + 100;
				//double radiusExternal = radiusEpoxyPhase + 10;
				double radiusExternal = radiusEpoxyPhase + 15;
				#endregion

				var newBallInternal = new Sphere(newCenter, radiusEpoxyPhase);
				var newBallExternal = new Sphere(newCenter, radiusExternal);

				if (CollidesWithOtherBalls(newBallInternal, newBallExternal, ballsInternal, ballsExternal)) continue;
				ballsInternal.Add(newBallInternal);
				ballsExternal.Add(newBallExternal);

				// Create phases
				var phaseInternal = new LsmPhase(GeometryModel.Phases.Count, GeometryModel, -1);
				GeometryModel.Phases[phaseInternal.ID] = phaseInternal;
				EpoxyPhaseIDs.Add(phaseInternal.ID);
				var phaseExternal = new HollowLsmPhase(GeometryModel.Phases.Count, GeometryModel, 0);
				GeometryModel.Phases[phaseExternal.ID] = phaseExternal;
				SilverPhaseIDs.Add(phaseExternal.ID);

				// Create phase boundaries
				var lsmExternal = new SimpleLsm3D(phaseExternal.ID, physicalModel.Nodes.Values.OrderBy(n => n.ID).ToList(), 
					newBallExternal);
				var boundaryExternal = new ClosedPhaseBoundary(phaseExternal.ID, lsmExternal, defaultPhase, phaseExternal);
				defaultPhase.ExternalBoundaries.Add(boundaryExternal);
				defaultPhase.Neighbors.Add(phaseExternal);
				phaseExternal.ExternalBoundaries.Add(boundaryExternal);
				phaseExternal.Neighbors.Add(defaultPhase);

				var lsmInternal = new SimpleLsm3D(phaseInternal.ID, physicalModel.Nodes.Values.OrderBy(n => n.ID).ToList(), 
					newBallInternal);
				var boundaryInternal = new ClosedPhaseBoundary(phaseInternal.ID, lsmInternal, phaseExternal, phaseInternal);
				phaseExternal.InternalBoundaries.Add(boundaryInternal);
				phaseExternal.Neighbors.Add(phaseInternal);
				phaseExternal.InternalPhases.Add(phaseInternal);
				phaseInternal.ExternalBoundaries.Add(boundaryInternal);
				phaseInternal.Neighbors.Add(phaseExternal);

				++b;
			}
		}

		private bool CollidesWithOtherBalls(Sphere newBallInternal, Sphere newBallExternal,
			List<Sphere> ballsInternal, List<Sphere> ballsExternal)
		{
			for (int i = 0; i < ballsInternal.Count; ++i)
			{
				double centerDistance = XFEM.Geometry.Utilities.Distance3D(newBallInternal.Center, ballsInternal[i].Center);
				//double centerDistance = XFEM.Geometry.Utilities.Distance2D(newBallInternal.Center, ballsInternal[i].Center);
				if (newBallExternal.Radius + ballsInternal[i].Radius >= centerDistance) return true;
				if (newBallInternal.Radius + ballsExternal[i].Radius >= centerDistance) return true;
			}
			return false;
		}
	}
}
