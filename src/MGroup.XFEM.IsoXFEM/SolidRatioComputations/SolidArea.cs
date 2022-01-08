namespace MGroup.XFEM.IsoXFEM.SolidRatioComputations
{
	using System;
	using System.Collections.Generic;
	using System.Text;

	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Reduction;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.XFEM.Elements;

	public class SolidArea : ISolidRatio
	{
		public XModel<IIsoXfemElement> ModelX { get; }
		public Vector InitialSizeOfElements { get; }
		public Vector RelativeCriteria { get; set; }
		public SolidArea(XModel<IIsoXfemElement> xModel, Vector initialSizeOfElements)
		{
			this.ModelX = xModel;
			this.InitialSizeOfElements = initialSizeOfElements;
		}

		public Vector CalculateSolidRatio()
		{
			Vector newArea = Vector.CreateZero(InitialSizeOfElements.Length);
			double areaRatio = 1.00;
			for (int i = 0; i < ModelX.Elements.Count; i++)
			{
				int[] connectionOfElement = new int[] { ModelX.Elements[i].Nodes[0].ID, ModelX.Elements[i].Nodes[1].ID, ModelX.Elements[i].Nodes[2].ID, ModelX.Elements[i].Nodes[3].ID };
				if (RelativeCriteria.GetSubvector(connectionOfElement).Min() > 0)//solid element
				{
					areaRatio = 1;
				}
				else if (RelativeCriteria.GetSubvector(connectionOfElement).Max() < 0)// void element
				{
					areaRatio = 0;
				}
				else //boundary element
				{
					Matrix s = Matrix.CreateZero(21, 21);
					Matrix t = Matrix.CreateZero(21, 21);
					for (int j = 0; j < 21; j++)
					{
						double v = -1;
						for (int k = 0; k < 21; k++)
						{
							t[k, j] = v;
							s[j, k] = v;
							v = v + 0.1;
						}
					}
					int m = 0;
					Vector tmpPhi = Vector.CreateZero(441);
					for (int j = 0; j < 21; j++)
					{
						for (int k = 0; k < 21; k++)
						{
							tmpPhi[m] = (1 - s[j, k]) * (1 - t[j, k]) * 0.25 * RelativeCriteria[connectionOfElement[0]] + (1 + s[j, k]) * (1 - t[j, k]) * 0.25 * RelativeCriteria[connectionOfElement[1]] + (1 + s[j, k]) * (1 + t[j, k]) * 0.25 * RelativeCriteria[connectionOfElement[2]] + (1 - s[j, k]) * (1 + t[j, k]) * 0.25 * RelativeCriteria[connectionOfElement[3]];
							m++;

						}
					}
					double numofNoNegative = 0;
					for (int j = 0; j < tmpPhi.Length; j++)
					{
						if (tmpPhi[j] >= 0.00)
						{
							numofNoNegative++;
						}
					}
					areaRatio = numofNoNegative / tmpPhi.Length;
				}
				newArea[i] = areaRatio * InitialSizeOfElements[i];
			}
			return newArea;
		}
	}
}
