namespace MGroup.XFEM.IsoXFEM.SolidRatioComputations
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.IsoXFEM.IsoXfemElements;

	public class SolidVolume : ISolidRatio
	{
		public XModel<IIsoXfemElement> ModelX { get; }
		public Vector InitialSizeOfElements { get; }
		public Vector RelativeCriteria { get; set; }
		public SolidVolume(XModel<IIsoXfemElement> xModel, Vector initialSizeOfElements)
		{
			this.ModelX = xModel;
			this.InitialSizeOfElements = initialSizeOfElements;
		}
		public Vector CalculateSolidRatio()
		{
			Vector newVolume = Vector.CreateZero(InitialSizeOfElements.Length);
			double volumeRatio = 1.00;
			for (int i = 0; i < ModelX.Elements.Count; i++)
			{
				var element = ModelX.Elements[i];
				int[] connectionOfElement = new int[element.Nodes.Count];
				int g = 0;
				foreach (var node in element.Nodes)
				{
					connectionOfElement[g] = node.ID;
					g++;
				}
				if (RelativeCriteria.GetSubvector(connectionOfElement).Min() > 0)//solid element
				{
					volumeRatio = 1;
				}
				else if (RelativeCriteria.GetSubvector(connectionOfElement).Max() < 0)// void element
				{
					volumeRatio = 0;
				}
				else //boundary element
				{
					Matrix s = Matrix.CreateZero(21, 21);
					Matrix t = Matrix.CreateZero(21, 21);
					for (int j = 0; j < 21; j++)
					{
						var v = -1.00;
						for (int k = 0; k < 21; k++)
						{
							t[k, j] = v;
							s[j, k] = v;
							v = v + 0.1;
						}
					}
					int m = 0;
					Vector tmpPhi = Vector.CreateZero(21*21*21);
					var l = -1.00;
					for (int b = 0; b < 21; b++)
					{
						Vector r = Vector.CreateWithValue(21, l);
						for (int j = 0; j < 21; j++)
						{
							for (int k = 0; k < 21; k++)
							{
								tmpPhi[m] = (((1 - r[b]) * (1 - s[j, k]) * (1 - t[j, k])) / 8) * RelativeCriteria[connectionOfElement[0]] + (((1 - r[b]) * (1 + s[j, k]) * (1 - t[j, k])) / 8) * RelativeCriteria[connectionOfElement[1]] +
									(((1 - r[b]) * (1 + s[j, k]) * (1 + t[j, k])) / 8) * RelativeCriteria[connectionOfElement[2]] + (((1 - r[b]) * (1 - s[j, k]) * (1 + t[j, k])) / 8) * RelativeCriteria[connectionOfElement[3]] +
									(((1 + r[b]) * (1 - s[j, k]) * (1 - t[j, k])) / 8) * RelativeCriteria[connectionOfElement[4]] + (((1 + r[b]) * (1 + s[j, k]) * (1 - t[j, k])) / 8) * RelativeCriteria[connectionOfElement[5]] +
									(((1 + r[b]) * (1 + s[j, k]) * (1 + t[j, k])) / 8) * RelativeCriteria[connectionOfElement[6]] + (((1 + r[b]) * (1 - s[j, k]) * (1 + t[j, k])) / 8) * RelativeCriteria[connectionOfElement[7]];
								m++;
							}
						}
						l = l + 0.1;
					}
					double numofNoNegative = 0;
					for (int j = 0; j < tmpPhi.Length; j++)
					{
						if (tmpPhi[j] >= 0.00)
						{
							numofNoNegative++;
						}
					}
					volumeRatio = numofNoNegative / tmpPhi.Length;
				}
				newVolume[i] = volumeRatio * InitialSizeOfElements[i];
			}
			return newVolume;
		}
	}
}

