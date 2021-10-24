using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Phases;

namespace MGroup.XFEM.Tests.MultiphaseThermal.EpoxyAg
{
    public class GeometryPreprocessorBase
    {
        private XModel<IXMultiphaseElement> physicalModel;

        protected GeometryPreprocessorBase(XModel<IXMultiphaseElement> physicalModel)
        {
            this.physicalModel = physicalModel;
        }

        public double[] MinCoordinates { get; set; }
        public double[] MaxCoordinates { get; set; }

        public int RngSeed { get; set; } = 33;

        public int NumBalls { get; set; } = 20;

        public PhaseGeometryModel GeometryModel { get; set; }

        public string MatrixPhaseName { get; } = "matrix";

        public int MatrixPhaseID { get; set; }

        public string EpoxyPhaseName { get; } = "epoxy";

        public List<int> EpoxyPhaseIDs { get; set; } = new List<int>();

        public string SilverPhaseName { get; } = "silver";

        public List<int> SilverPhaseIDs { get; set; } = new List<int>();

        public Dictionary<string, double> CalcPhaseVolumes()
        {
            var volumes = new Dictionary<string, double>();
            Dictionary<int, double> phaseVolumes = XFEM.Output.Fields.Utilities.CalcBulkSizeOfEachPhase(physicalModel, GeometryModel);

            volumes[MatrixPhaseName] = phaseVolumes[MatrixPhaseID];

            volumes[EpoxyPhaseName] = 0;
            foreach (int phaseID in EpoxyPhaseIDs) volumes[EpoxyPhaseName] += phaseVolumes[phaseID];

            volumes[SilverPhaseName] = 0;
            foreach (int phaseID in SilverPhaseIDs)
            {
                try
                {
                    volumes[SilverPhaseName] += phaseVolumes[phaseID];
                }
                catch (KeyNotFoundException)
                {
                    // This phase has been merged into another one. Nothing more to do here.
                }
            }

            return volumes;
        }

        public string PrintVolumes()
        {
            var stringBuilder = new StringBuilder();
            Dictionary<string, double> volumes = CalcPhaseVolumes();
            stringBuilder.AppendLine();
            stringBuilder.Append("Volumes of each material: ");
            foreach (string phase in volumes.Keys)
            {
                stringBuilder.Append($"{phase} phase : {volumes[phase]}, ");
            }
            stringBuilder.AppendLine();

            double totalVolume = 0;
            foreach (string phase in volumes.Keys)
            {
                totalVolume += volumes[phase];
            }
            stringBuilder.AppendLine($"Total volume: {totalVolume}");

            double volFracAg = volumes[SilverPhaseName] / totalVolume;
            stringBuilder.AppendLine($"Volume fraction Ag: {volFracAg}");
            double volFracInclusions =
                (volumes[SilverPhaseName] + volumes[EpoxyPhaseName]) / totalVolume;
            stringBuilder.AppendLine($"Volume fraction inclusions: {volFracInclusions}");

            return stringBuilder.ToString();
        }
    }
}
