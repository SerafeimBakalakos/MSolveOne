using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.ContinuumElements;
using MGroup.Environments;
using MGroup.FEM.Entities;
using MGroup.FEM.Structural.Elements;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Loads;
using MGroup.Solvers.DDM.Mesh;
using MGroup.Solvers.DDM.Partitioning;

//TODO: different number of clusters, subdomains, elements per axis. Try to make this as nonsymmetric as possible, 
//      but keep subdomain-elements ratio constant to have the same stiffnesses.
//TODO: Finding the correct indexing data by hand is going to be very difficult. Take them from a correct solution and 
//      hardcode them.
namespace MGroup.Solvers.DDM.Tests.ExampleModels
{
	public class Brick3DExample
	{
		private const double E = 1.0, v = 0.3;
		private const double load = 100;

		public static double[] MinCoords => new double[] { 0, 0, 0 };

		public static double[] MaxCoords => new double[] { 6, 9, 12 };

		public static int[] NumElements => new int[] { 4, 6, 8 };

		public static int[] NumSubdomains => new int[] { 2, 3, 4 };

		public static int[] NumClusters => new int[] { 2, 1, 2 };

		public static ComputeNodeTopology CreateNodeTopology()
		{
			var nodeTopology = new ComputeNodeTopology();
			Dictionary<int, int> clustersOfSubdomains = GetSubdomainClusters();
			Dictionary<int, int[]> neighborsOfSubdomains = GetSubdomainNeighbors();
			for (int s = 0; s < NumSubdomains[0] * NumSubdomains[1] * NumSubdomains[2]; ++s)
			{
				nodeTopology.AddNode(s, neighborsOfSubdomains[s], clustersOfSubdomains[s]);
			}
			return nodeTopology;
		}

		public static IModel/*DistributedModel*/ CreateSingleSubdomainDistributedModel(IComputeEnvironment environment)
		{
			throw new NotImplementedException();
			//AllDofs.AddDof(StructuralDof.TranslationX);
   //         AllDofs.AddDof(StructuralDof.TranslationY);
   //         AllDofs.AddDof(StructuralDof.TranslationZ);
   //         var model = new DistributedModel(environment);
   //         model.SubdomainsDictionary[0] = new Subdomain(0);

   //         var mesh = new UniformMesh3D.Builder(MinCoords, MaxCoords, NumElements).SetMajorMinorAxis(0, 2).BuildMesh();

   //         // Nodes
   //         foreach ((int id, double[] coords) in mesh.EnumerateNodes())
   //         {
   //             model.NodesDictionary[id] = new Node(id, coords[0], coords[1], coords[2]);
   //         }

   //         // Materials
   //         var material = new ElasticMaterial3D() { YoungModulus = E, PoissonRatio = v };
   //         var dynamicProperties = new DynamicMaterial(1.0, 1.0, 1.0);

   //         // Elements
   //         var elemFactory = new ContinuumElement3DFactory(material, dynamicProperties);
   //         foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
   //         {
   //             Node[] nodes = nodeIDs.Select(n => model.NodesDictionary[n]).ToArray();
   //             var elementType = elemFactory.CreateElement(mesh.CellType, nodes);
   //             var element = new Element() { ID = elementID, ElementType = elementType };
   //             foreach (var node in nodes) element.AddNode(node);
   //             model.ElementsDictionary[element.ID] = element;
   //             model.SubdomainsDictionary[0].Elements.Add(element);
   //         }

   //         // Boundary conditions
   //         //TODO: hardcode the node IDs
   //         var constrainedNodes = new List<int>();
   //         constrainedNodes.Add(/*mesh.GetNodeID(new int[] { 0, 0, 0 })*/0 );
   //         constrainedNodes.Add(/*mesh.GetNodeID(new int[] { mesh.NumNodes[0] - 1, 0, 0 })/*6*/4);
   //         constrainedNodes.Add(/*mesh.GetNodeID(new int[] { 0, mesh.NumNodes[1] - 1, 0 })*//*63*/30);
   //         foreach (int nodeID in constrainedNodes)
   //         {
   //             Node node = model.NodesDictionary[nodeID];
   //             node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
   //             node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
   //             node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
   //         }

   //         var loadedNodes = new List<int>();
   //         loadedNodes.Add(/*mesh.GetNodeID(new int[] { mesh.NumNodes[0] - 1, mesh.NumNodes[1] - 1, mesh.NumNodes[2] - 1 })*//*909*/314);
   //         foreach (int nodeID in loadedNodes)
   //         {
   //             Node node = model.NodesDictionary[nodeID];
   //             model.Loads.Add(new Load() { Node = node, DOF = StructuralDof.TranslationZ, Amount = load });
   //         }

   //         return model;
		}

		//TODOMPI: Remove this
		public static Model CreateSingleSubdomainModel()
		{
			AllDofs.AddDof(StructuralDof.TranslationX);
			AllDofs.AddDof(StructuralDof.TranslationY);
			AllDofs.AddDof(StructuralDof.TranslationZ);
			var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(0);

			var mesh = new UniformMesh3D.Builder(MinCoords, MaxCoords, NumElements).SetMajorMinorAxis(0, 2).BuildMesh();

			// Nodes
			foreach ((int id, double[] coords) in mesh.EnumerateNodes())
			{
				model.NodesDictionary[id] = new Node(id, coords[0], coords[1], coords[2]);
			}

			// Materials
			var material = new ElasticMaterial3D() { YoungModulus = E, PoissonRatio = v };
			var dynamicProperties = new DynamicMaterial(1.0, 1.0, 1.0);

			// Elements
			var elemFactory = new ContinuumElement3DFactory(material, dynamicProperties);
			foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
			{
				Node[] nodes = nodeIDs.Select(n => model.NodesDictionary[n]).ToArray();
				var elementType = elemFactory.CreateElement(mesh.CellType, nodes);
				var element = new Element() { ID = elementID, ElementType = elementType };
				foreach (var node in nodes) element.AddNode(node);
				model.ElementsDictionary[element.ID] = element;
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			// Boundary conditions
			//TODO: hardcode the node IDs
			var constrainedNodes = new List<int>();
			constrainedNodes.Add(/*mesh.GetNodeID(new int[] { 0, 0, 0 })*/0);
			constrainedNodes.Add(/*mesh.GetNodeID(new int[] { mesh.NumNodes[0] - 1, 0, 0 })/*6*/4);
			constrainedNodes.Add(/*mesh.GetNodeID(new int[] { 0, mesh.NumNodes[1] - 1, 0 })*//*63*/30);
			foreach (int nodeID in constrainedNodes)
			{
				Node node = model.NodesDictionary[nodeID];
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationZ, Amount = 0 });
			}

			var loadedNodes = new List<int>();
			loadedNodes.Add(/*mesh.GetNodeID(new int[] { mesh.NumNodes[0] - 1, mesh.NumNodes[1] - 1, mesh.NumNodes[2] - 1 })*//*909*/314);
			foreach (int nodeID in loadedNodes)
			{
				Node node = model.NodesDictionary[nodeID];
				model.Loads.Add(new Load() { Node = node, DOF = StructuralDof.TranslationZ, Amount = load });
			}

			return model;
		}

		public static IModel CreateMultiSubdomainModel(IComputeEnvironment environment)
		{
			throw new NotImplementedException();
			//Dictionary<int, int> elementsToSubdomains = GetSubdomainsOfElements();
			//DistributedModel model = CreateSingleSubdomainDistributedModel(environment);
			//model.DecomposeIntoSubdomains(NumSubdomains[0] * NumSubdomains[1] * NumSubdomains[2], e => elementsToSubdomains[e]);
			//return model;
		}

		public static Table<int, int, double> GetExpectedNodalValues()
		{
			var result = new Table<int, int, double>();
			#region long list of solution values per dof
			result[0, 0] = 0; result[0, 1] = 0; result[0, 2] = 0;
			result[1, 0] = 50.922197735894756; result[1, 1] = 17.9755448390772; result[1, 2] = -214.64651928346373;
			result[2, 0] = 67.66259154588649; result[2, 1] = 66.98508867950706; result[2, 2] = -24.789749258130566;
			result[3, 0] = 47.87485771328109; result[3, 1] = 116.28996057949249; result[3, 2] = 174.1355419208071;
			result[4, 0] = 0; result[4, 1] = 0; result[4, 2] = 0;
			result[5, 0] = 43.9257432853672; result[5, 1] = 8.396084637338213; result[5, 2] = -271.70186819630567;
			result[6, 0] = 48.6177056802344; result[6, 1] = 24.93499715188943; result[6, 2] = -50.884836062275845;
			result[7, 0] = 53.97232337432083; result[7, 1] = 68.66468339825879; result[7, 2] = 120.34542062694524;
			result[8, 0] = 59.46447675017824; result[8, 1] = 98.46378577221262; result[8, 2] = 292.4200336904981;
			result[9, 0] = 47.546908199408364; result[9, 1] = 95.24361708543412; result[9, 2] = 510.8077103197342;
			result[10, 0] = 63.25666551024514; result[10, 1] = 20.883384237658635; result[10, 2] = -142.67985307622726;
			result[11, 0] = 59.09697248041097; result[11, 1] = 36.39446177599841; result[11, 2] = 59.04846007667131;
			result[12, 0] = 58.86547764291654; result[12, 1] = 67.15292227957566; result[12, 2] = 264.95674456037335;
			result[13, 0] = 61.1086426996612; result[13, 1] = 95.633189365988; result[13, 2] = 470.65599127407495;
			result[14, 0] = 64.00652592075276; result[14, 1] = 109.81854240214388; result[14, 2] = 673.0239530019066;
			result[15, 0] = 55.78320481447402; result[15, 1] = 24.49789814481318; result[15, 2] = -20.45060213688484;
			result[16, 0] = 55.86843434861002; result[16, 1] = 41.2655139854666; result[16, 2] = 194.527523673131;
			result[17, 0] = 56.11356204154401; result[17, 1] = 67.25556381726635; result[17, 2] = 410.29925430922725;
			result[18, 0] = 56.43801040768812; result[18, 1] = 93.6124846278834; result[18, 2] = 625.9693574375175;
			result[19, 0] = 55.993559737654664; result[19, 1] = 111.16695224659368; result[19, 2] = 843.2201880529643;
			result[20, 0] = 43.621844835787215; result[20, 1] = 18.217014711088858; result[20, 2] = 103.29313033278129;
			result[21, 0] = 48.94093761138207; result[21, 1] = 37.01708945083352; result[21, 2] = 330.85234973387946;
			result[22, 0] = 47.98677455699289; result[22, 1] = 66.32871785196974; result[22, 2] = 552.9140156762392;
			result[23, 0] = 46.39862716835806; result[23, 1] = 92.78103569590203; result[23, 2] = 777.5475896657557;
			result[24, 0] = 45.53402087819768; result[24, 1] = 112.75424871524999; result[24, 2] = 1000.1288296513529;
			result[25, 0] = 56.214537862158394; result[25, 1] = 3.7452128677013827; result[25, 2] = 241.34551673921814;
			result[26, 0] = 42.62116387907786; result[26, 1] = 32.18686650265326; result[26, 2] = 441.58225556404875;
			result[27, 0] = 37.04661225432036; result[27, 1] = 65.7625661673989; result[27, 2] = 696.5158347114835;
			result[28, 0] = 31.467285084843674; result[28, 1] = 92.6782072502821; result[28, 2] = 925.1962106983615;
			result[29, 0] = 30.017612164853837; result[29, 1] = 113.49158184783475; result[29, 2] = 1152.5261240721895;
			result[30, 0] = 0; result[30, 1] = 0; result[30, 2] = 0;
			result[31, 0] = 22.61072150582391; result[31, 1] = 19.437273282814317; result[31, 2] = 604.400367164636;
			result[32, 0] = 15.851054443334899; result[32, 1] = 70.14538755065638; result[32, 2] = 833.6030901727183;
			result[33, 0] = 11.157941840508473; result[33, 1] = 92.46090112926737; result[33, 2] = 1072.5250245780794;
			result[34, 0] = 10.42458733216261; result[34, 1] = 114.15047975645629; result[34, 2] = 1299.945312272757;
			result[35, 0] = -108.44049403578094; result[35, 1] = -108.25752646962336; result[35, 2] = -317.0825148489985;
			result[36, 0] = -86.51444373280957; result[36, 1] = -94.24209706303351; result[36, 2] = -186.65458880437737;
			result[37, 0] = -106.63357266409837; result[37, 1] = -75.50490960955426; result[37, 2] = -24.239334637118194;
			result[38, 0] = -92.86992635929467; result[38, 1] = -65.41550329335548; result[38, 2] = 144.97642351155113;
			result[39, 0] = -116.26649765648577; result[39, 1] = -42.23411324687882; result[39, 2] = 283.31066463244116;
			result[40, 0] = -130.55176441685282; result[40, 1] = -82.701141639971; result[40, 2] = -241.17162398635986;
			result[41, 0] = -126.55488132154827; result[41, 1] = -92.02464775734637; result[41, 2] = -67.2234711447693;
			result[42, 0] = -123.20763004226183; result[42, 1] = -75.51415778680524; result[42, 2] = 119.15540593687683;
			result[43, 0] = -125.56891148043965; result[43, 1] = -61.693588425314715; result[43, 2] = 306.1579437667135;
			result[44, 0] = -129.11159493929813; result[44, 1] = -76.44967997549779; result[44, 2] = 474.4174601048281;
			result[45, 0] = -145.8208623302301; result[45, 1] = -105.43342233834505; result[45, 2] = -149.6485416276871;
			result[46, 0] = -142.32826408643712; result[46, 1] = -91.62415025850693; result[46, 2] = 59.24577809885651;
			result[47, 0] = -142.18486963934603; result[47, 1] = -75.34947746254969; result[47, 2] = 264.4124329022108;
			result[48, 0] = -140.96946224071812; result[48, 1] = -60.50169168540455; result[48, 2] = 468.55394972550255;
			result[49, 0] = -145.46100569701954; result[49, 1] = -48.091423928558285; result[49, 2] = 677.6318389738061;
			result[50, 0] = -158.72140742615082; result[50, 1] = -108.86185946465282; result[50, 2] = -20.630945897492815;
			result[51, 0] = -158.04465641830473; result[51, 1] = -93.33315506116294; result[51, 2] = 194.31724708758807;
			result[52, 0] = -157.8489795152322; result[52, 1] = -74.63335359276108; result[52, 2] = 410.1227007417851;
			result[53, 0] = -158.4445650424871; result[53, 1] = -56.06378814099574; result[53, 2] = 626.3755682560261;
			result[54, 0] = -159.915529267852; result[54, 1] = -39.35052327385978; result[54, 2] = 841.5777194288856;
			result[55, 0] = -173.42436114908986; result[55, 1] = -108.94844575831739; result[55, 2] = 110.2834410767148;
			result[56, 0] = -175.98712982823136; result[56, 1] = -91.28636853820977; result[56, 2] = 329.5397227631788;
			result[57, 0] = -175.74615254607053; result[57, 1] = -73.91074055981704; result[57, 2] = 554.2312229218331;
			result[58, 0] = -176.406999704888; result[58, 1] = -53.71401569412017; result[58, 2] = 777.8745772751224;
			result[59, 0] = -177.40243262067364; result[59, 1] = -34.93072275100292; result[59, 2] = 1000.1216735236978;
			result[60, 0] = -198.13009286428587; result[60, 1] = -89.6824497229456; result[60, 2] = 209.25468485291955;
			result[61, 0] = -197.96499345677177; result[61, 1] = -91.96010304921751; result[61, 2] = 456.96470477878137;
			result[62, 0] = -198.5447278354371; result[62, 1] = -72.9328367130458; result[62, 2] = 695.6452870396529;
			result[63, 0] = -196.56724475826803; result[63, 1] = -53.06628517766403; result[63, 2] = 926.5230483376528;
			result[64, 0] = -196.96535166746344; result[64, 1] = -32.57405676825995; result[64, 2] = 1152.4366593230006;
			result[65, 0] = -215.69030286499813; result[65, 1] = -115.94576241557621; result[65, 2] = 291.68577802721217;
			result[66, 0] = -243.82812503645874; result[66, 1] = -93.30309806339729; result[66, 2] = 571.362719955756;
			result[67, 0] = -221.491908519286; result[67, 1] = -76.2429634801263; result[67, 2] = 840.4991408594915;
			result[68, 0] = -218.4675836452834; result[68, 1] = -53.14682346436332; result[68, 2] = 1071.8126938472487;
			result[69, 0] = -217.9139951379428; result[69, 1] = -32.0915005998845; result[69, 2] = 1300.1713228044282;
			result[70, 0] = -309.15497051497977; result[70, 1] = -259.96716500801926; result[70, 2] = -409.45212822219685;
			result[71, 0] = -299.56066746041733; result[71, 1] = -237.76843094782984; result[71, 2] = -212.31147479304983;
			result[72, 0] = -295.10380794026474; result[72, 1] = -219.35243468109232; result[72, 2] = -25.61429410982252;
			result[73, 0] = -299.36290445736887; result[73, 1] = -199.7445700382937; result[73, 2] = 162.89704021302413;
			result[74, 0] = -305.5687217330426; result[74, 1] = -178.33780693368183; result[74, 2] = 359.783027560013;
			result[75, 0] = -329.4988373081947; result[75, 1] = -250.77222373782652; result[75, 2] = -272.6411386398248;
			result[76, 0] = -322.05350613848105; result[76, 1] = -232.04131902512918; result[76, 2] = -75.8856383732122;
			result[77, 0] = -321.3185529180286; result[77, 1] = -218.16479953086994; result[77, 2] = 118.41970862562935;
			result[78, 0] = -321.67712956930654; result[78, 1] = -204.0278477475847; result[78, 2] = 312.36843203232513;
			result[79, 0] = -329.0649562325234; result[79, 1] = -183.4284066871534; result[79, 2] = 509.2074870119579;
			result[80, 0] = -351.68849955359025; result[80, 1] = -248.46540026455406; result[80, 2] = -150.5664855486981;
			result[81, 0] = -348.59993277163943; result[81, 1] = -232.50664632790813; result[81, 2] = 56.799759473204546;
			result[82, 0] = -347.3471394487668; result[82, 1] = -217.4776591358973; result[82, 2] = 263.95942703776814;
			result[83, 0] = -348.747744818727; result[83, 1] = -202.27546510102604; result[83, 2] = 470.8834081781303;
			result[84, 0] = -352.29506321006244; result[84, 1] = -185.61108001043138; result[84, 2] = 677.6870796013512;
			result[85, 0] = -373.01359333915906; result[85, 1] = -251.82531377433185; result[85, 2] = -22.444991192355058;
			result[86, 0] = -372.0757338038895; result[86, 1] = -232.93831090705484; result[86, 2] = 193.51806779266246;
			result[87, 0] = -372.0328791951983; result[87, 1] = -216.44929687746836; result[87, 2] = 410.26230885679087;
			result[88, 0] = -372.9984598039153; result[88, 1] = -199.33683052671304; result[88, 2] = 626.8114639220503;
			result[89, 0] = -375.67085018344005; result[89, 1] = -179.21542658334903; result[89, 2] = 843.5470305798735;
			result[90, 0] = -395.0100502226465; result[90, 1] = -249.2124441272309; result[90, 2] = 107.26795190670644;
			result[91, 0] = -395.92194514006263; result[91, 1] = -232.0181863687371; result[91, 2] = 330.5442261378406;
			result[92, 0] = -396.59026056461323; result[92, 1] = -215.27835899164944; result[92, 2] = 555.2141441005374;
			result[93, 0] = -397.2715756549937; result[93, 1] = -197.00290538064343; result[93, 2] = 779.197164005205;
			result[94, 0] = -398.8338550614058; result[94, 1] = -175.95490160980455; result[94, 2] = 1000.9612954940737;
			result[95, 0] = -417.4524090810113; result[95, 1] = -250.90764079263036; result[95, 2] = 233.2428096641266;
			result[96, 0] = -422.848336644583; result[96, 1] = -231.05260649060673; result[96, 2] = 463.1796043553014;
			result[97, 0] = -422.1915970959725; result[97, 1] = -215.70204673997296; result[97, 2] = 698.3611623144892;
			result[98, 0] = -421.612274784597; result[98, 1] = -196.5252511755765; result[98, 2] = 928.1668715629339;
			result[99, 0] = -422.12655226805634; result[99, 1] = -175.09919206891064; result[99, 2] = 1153.946889645317;
			result[100, 0] = -438.69033549684747; result[100, 1] = -256.73170360099755; result[100, 2] = 369.64697063749037;
			result[101, 0] = -444.65865822602615; result[101, 1] = -237.04530453053002; result[101, 2] = 599.4968020142967;
			result[102, 0] = -447.16962259804797; result[102, 1] = -217.97106267810997; result[102, 2] = 840.1738513650944;
			result[103, 0] = -444.3806244370263; result[103, 1] = -197.97579575749887; result[103, 2] = 1074.6157810048583;
			result[104, 0] = -444.89938318659676; result[104, 1] = -175.74163369270153; result[104, 2] = 1301.1876459068596;
			result[105, 0] = -516.7963608932072; result[105, 1] = -412.9588083482648; result[105, 2] = -443.476443437731;
			result[106, 0] = -510.21993551167543; result[106, 1] = -387.9830868385917; result[106, 2] = -234.83167644567718;
			result[107, 0] = -507.7119604371998; result[107, 1] = -364.06891340584815; result[107, 2] = -29.727217832945062;
			result[108, 0] = -508.04044683891345; result[108, 1] = -339.7467577146955; result[108, 2] = 175.3507546226506;
			result[109, 0] = -512.1477400503748; result[109, 1] = -312.3869598224287; result[109, 2] = 383.5724136784895;
			result[110, 0] = -540.7492052682069; result[110, 1] = -406.4049534625002; result[110, 2] = -298.76178929465715;
			result[111, 0] = -535.4950244953905; result[111, 1] = -383.7066260828926; result[111, 2] = -88.63542704749298;
			result[112, 0] = -533.1932155469775; result[112, 1] = -362.88059792046556; result[112, 2] = 116.65110825808557;
			result[113, 0] = -534.6823657254374; result[113, 1] = -341.4425555219345; result[113, 2] = 322.0032735476348;
			result[114, 0] = -538.9369221287235; result[114, 1] = -317.0879381619898; result[114, 2] = 531.6949778632882;
			result[115, 0] = -565.0447077662446; result[115, 1] = -403.43188081246797; result[115, 2] = -161.82013890614988;
			result[116, 0] = -561.9395498482967; result[116, 1] = -381.3528406187113; result[116, 2] = 52.01685358263141;
			result[117, 0] = -561.1949602316938; result[117, 1] = -362.0007563681938; result[117, 2] = 263.64356127469;
			result[118, 0] = -562.2913244092351; result[118, 1] = -342.08658666231605; result[118, 2] = 475.1792542280515;
			result[119, 0] = -565.9091269785949; result[119, 1] = -318.19307471223254; result[119, 2] = 689.0325480165693;
			result[120, 0] = -589.6916274768047; result[120, 1] = -401.92997373476464; result[120, 2] = -26.314668552993844;
			result[121, 0] = -588.6495266802177; result[121, 1] = -380.4798316798842; result[121, 2] = 192.2680421675966;
			result[122, 0] = -588.7539265032387; result[122, 1] = -361.0495790381262; result[122, 2] = 411.0266136227567;
			result[123, 0] = -590.1630253718257; result[123, 1] = -340.929794549617; result[123, 2] = 629.6999075165576;
			result[124, 0] = -592.9383085520374; result[124, 1] = -317.480097809357; result[124, 2] = 848.1951219728622;
			result[125, 0] = -613.8191244892316; result[125, 1] = -401.68284882003786; result[125, 2] = 110.08227981873458;
			result[126, 0] = -614.7620266669986; result[126, 1] = -380.01235090162487; result[126, 2] = 332.7249543362268;
			result[127, 0] = -615.6373967763052; result[127, 1] = -360.50315104966086; result[127, 2] = 558.0435230164044;
			result[128, 0] = -617.164403189857; result[128, 1] = -339.948971091149; result[128, 2] = 782.7403025364447;
			result[129, 0] = -619.5717167167716; result[129, 1] = -316.31588730216544; result[129, 2] = 1005.4281354500682;
			result[130, 0] = -636.7231849942982; result[130, 1] = -402.36268756732034; result[130, 2] = 248.65055040061083;
			result[131, 0] = -639.5623442031161; result[131, 1] = -381.44363098094254; result[131, 2] = 473.79504444424884;
			result[132, 0] = -641.6875827079841; result[132, 1] = -361.3230252651464; result[132, 2] = 704.0398412753408;
			result[133, 0] = -643.1668295662021; result[133, 1] = -340.54099917604185; result[133, 2] = 933.4607977813191;
			result[134, 0] = -645.3482641442434; result[134, 1] = -317.1095083242753; result[134, 2] = 1158.999593546596;
			result[135, 0] = -657.5661664807868; result[135, 1] = -406.19516200672916; result[135, 2] = 393.0115712909252;
			result[136, 0] = -662.2237197139825; result[136, 1] = -385.0615650024591; result[136, 2] = 619.2642264711818;
			result[137, 0] = -664.8316805492183; result[137, 1] = -364.30586326674944; result[137, 2] = 850.4605757543302;
			result[138, 0] = -667.2455431108596; result[138, 1] = -342.8152028556309; result[138, 2] = 1080.872118878769;
			result[139, 0] = -669.6466407355708; result[139, 1] = -319.4944574350215; result[139, 2] = 1307.4736017741693;
			result[140, 0] = -731.2372008159687; result[140, 1] = -566.285550149535; result[140, 2] = -464.7088975645027;
			result[141, 0] = -727.1962230559633; result[141, 1] = -539.1075460947034; result[141, 2] = -248.44054799456563;
			result[142, 0] = -725.0572418980151; result[142, 1] = -512.0518724929176; result[142, 2] = -34.471072574311194;
			result[143, 0] = -724.8519294162007; result[143, 1] = -484.39092173047004; result[143, 2] = 179.39168043964037;
			result[144, 0] = -726.3237196245608; result[144, 1] = -455.2433569111271; result[144, 2] = 395.6493380961883;
			result[145, 0] = -757.792739023881; result[145, 1] = -561.8666638303907; result[145, 2] = -315.89324892476185;
			result[146, 0] = -754.2291398120551; result[146, 1] = -536.0192928649087; result[146, 2] = -99.22997815518015;
			result[147, 0] = -752.6667244827015; result[147, 1] = -510.827620107346; result[147, 2] = 114.47933747404775;
			result[148, 0] = -753.1977090761919; result[148, 1] = -485.08592386470383; result[148, 2] = 328.1223833724118;
			result[149, 0] = -755.774060274764; result[149, 1] = -457.33096337927157; result[149, 2] = 544.7294733727408;
			result[150, 0] = -784.0176218479776; result[150, 1] = -558.7211228001521; result[150, 2] = -172.84732335520795;
			result[151, 0] = -781.6710039684144; result[151, 1] = -533.721239971761; result[151, 2] = 46.29708233548891;
			result[152, 0] = -780.9419278769354; result[152, 1] = -509.84002571443824; result[152, 2] = 263.1586844815226;
			result[153, 0] = -781.9650695796835; result[153, 1] = -485.3309209847675; result[153, 2] = 479.9846863231244;
			result[154, 0] = -784.6385967961919; result[154, 1] = -458.4582520862555; result[154, 2] = 698.9278555729833;
			result[155, 0] = -810.0353956214636; result[155, 1] = -556.787139240982; result[155, 2] = -31.420966360983755;
			result[156, 0] = -809.0039162048278; result[156, 1] = -532.2816575520726; result[156, 2] = 190.60734802153556;
			result[157, 0] = -809.147953551496; result[157, 1] = -508.90928929638926; result[157, 2] = 412.27487550227215;
			result[158, 0] = -810.5184690443127; result[158, 1] = -484.7905606108231; result[158, 2] = 633.9211181036097;
			result[159, 0] = -813.2926702910822; result[159, 1] = -458.28060270271015; result[159, 2] = 855.7722951200694;
			result[160, 0] = -835.1145317524131; result[160, 1] = -555.6467574210305; result[160, 2] = 110.61881910637555;
			result[161, 0] = -835.3438444670319; result[161, 1] = -531.8946525877922; result[161, 2] = 335.4099559931488;
			result[162, 0] = -836.3108590846068; result[162, 1] = -508.5551297130784; result[162, 2] = 562.0059093443333;
			result[163, 0] = -838.2280829249183; result[163, 1] = -484.4849509801514; result[163, 2] = 788.6974924953195;
			result[164, 0] = -841.3522054433568; result[164, 1] = -458.444784106826; result[164, 2] = 1013.3654079685898;
			result[165, 0] = -858.7078351792071; result[165, 1] = -555.8182553333753; result[165, 2] = 254.96364881792147;
			result[166, 0] = -860.1158072306202; result[166, 1] = -532.8659646690209; result[166, 2] = 481.9183225372855;
			result[167, 0] = -861.9813354473388; result[167, 1] = -509.7924036029123; result[167, 2] = 712.1401797193068;
			result[168, 0] = -864.6496889979326; result[168, 1] = -485.90918945509037; result[168, 2] = 942.4958519311733;
			result[169, 0] = -868.4139774604832; result[169, 1] = -460.50510383649123; result[169, 2] = 1169.4948475080091;
			result[170, 0] = -880.891506624111; result[170, 1] = -557.1815565498089; result[170, 2] = 404.05186626249724;
			result[171, 0] = -883.1973379563707; result[171, 1] = -535.3808827423663; result[171, 2] = 631.4444196828331;
			result[172, 0] = -885.9898554950956; result[172, 1] = -512.7767718280663; result[172, 2] = 862.007406117013;
			result[173, 0] = -889.4887170701693; result[173, 1] = -489.39364305316576; result[173, 2] = 1092.7429686910868;
			result[174, 0] = -894.1846709686329; result[174, 1] = -464.7127499738378; result[174, 2] = 1320.2397927062646;
			result[175, 0] = -950.9239705552658; result[175, 1] = -720.5151958888416; result[175, 2] = -478.1256352694338;
			result[176, 0] = -948.3383523347445; result[176, 1] = -691.6814221114755; result[176, 2] = -257.5108403038448;
			result[177, 0] = -946.873260693239; result[177, 1] = -662.8350316979663; result[177, 2] = -38.61101557243818;
			result[178, 0] = -946.391585009902; result[178, 1] = -633.531992062616; result[178, 2] = 180.13539651467573;
			result[179, 0] = -946.9426321500307; result[179, 1] = -603.1854178188187; result[179, 2] = 400.95079436854667;
			result[180, 0] = -979.2041956571754; result[180, 1] = -717.5978117695211; result[180, 2] = -327.50227713492404;
			result[181, 0] = -976.9389383240036; result[181, 1] = -689.633017868615; result[181, 2] = -106.47253875116785;
			result[182, 0] = -975.8258052110441; result[182, 1] = -661.8497899325355; result[182, 2] = 112.32787910464153;
			result[183, 0] = -975.990984126009; result[183, 1] = -633.5939724623881; result[183, 2] = 331.07919414978534;
			result[184, 0] = -977.3406543267977; result[184, 1] = -604.1488420129995; result[184, 2] = 552.18345097998;
			result[185, 0] = -1007.1354232468555; result[185, 1] = -715.2436283452736; result[185, 2] = -181.39136347988216;
			result[186, 0] = -1005.4783907277806; result[186, 1] = -687.8986851956968; result[186, 2] = 41.56771529726331;
			result[187, 0] = -1004.9174314286734; result[187, 1] = -660.9356163135197; result[187, 2] = 262.45632739794195;
			result[188, 0] = -1005.5294134322752; result[188, 1] = -633.382708719057; result[188, 2] = 483.25315985358736;
			result[189, 0] = -1007.3696577343331; result[189, 1] = -604.4344344117316; result[189, 2] = 705.8065192965113;
			result[190, 0] = -1034.5488145523593; result[190, 1] = -713.3029076715789; result[190, 2] = -36.724876352270925;
			result[191, 0] = -1033.6002851981832; result[191, 1] = -686.6060502805265; result[191, 2] = 188.76170140922196;
			result[192, 0] = -1033.5641994569812; result[192, 1] = -659.9477435086983; result[192, 2] = 413.2340959134017;
			result[193, 0] = -1034.6523975463742; result[193, 1] = -632.4090899984719; result[193, 2] = 637.7575656956235;
			result[194, 0] = -1036.9969985595158; result[194, 1] = -603.528317594143; result[194, 2] = 862.8575212582948;
			result[195, 0] = -1060.7600099056174; result[195, 1] = -711.8470263927653; result[195, 2] = 108.53305090569891;
			result[196, 0] = -1060.3827440098669; result[196, 1] = -685.9750537115449; result[196, 2] = 336.7659525826029;
			result[197, 0] = -1060.7308560895679; result[197, 1] = -659.397272351548; result[197, 2] = 565.6617349031907;
			result[198, 0] = -1062.1655017870762; result[198, 1] = -631.4762421876574; result[198, 2] = 795.4682871175243;
			result[199, 0] = -1065.5979600225237; result[199, 1] = -603.0975138367908; result[199, 2] = 1023.8839543879337;
			result[200, 0] = -1085.4045376163867; result[200, 1] = -711.1335301266591; result[200, 2] = 256.0073148919006;
			result[201, 0] = -1085.374721768517; result[201, 1] = -686.4263488349135; result[201, 2] = 486.69172347872126;
			result[202, 0] = -1085.8221383497123; result[202, 1] = -660.2198692069242; result[202, 2] = 719.9574022964337;
			result[203, 0] = -1088.1642241028935; result[203, 1] = -632.958491810904; result[203, 2] = 955.0466000375958;
			result[204, 0] = -1093.4966774978204; result[204, 1] = -605.4707155224448; result[204, 2] = 1186.0080174540294;
			result[205, 0] = -1108.8210884315158; result[205, 1] = -711.323660755969; result[205, 2] = 407.7854069762592;
			result[206, 0] = -1109.2289064977415; result[206, 1] = -688.0197792573376; result[206, 2] = 639.3509443357237;
			result[207, 0] = -1110.4506435370142; result[207, 1] = -663.2809743849647; result[207, 2] = 873.7933276674635;
			result[208, 0] = -1113.7713118990475; result[208, 1] = -637.9503675793363; result[208, 2] = 1109.706868138546;
			result[209, 0] = -1120.4704513116021; result[209, 1] = -611.8773581488617; result[209, 2] = 1341.9528302947983;
			result[210, 0] = -1173.7254992314422; result[210, 1] = -875.2971332650636; result[210, 2] = -485.92209920444776;
			result[211, 0] = -1172.3893767577933; result[211, 1] = -845.3913021410394; result[211, 2] = -262.9301906768495;
			result[212, 0] = -1171.5149045245712; result[212, 1] = -815.7122073525535; result[212, 2] = -41.46992071924802;
			result[213, 0] = -1171.0642593514551; result[213, 1] = -785.7909499805062; result[213, 2] = 179.81392693092312;
			result[214, 0] = -1171.116086332369; result[214, 1] = -755.190641443248; result[214, 2] = 403.01431728471334;
			result[215, 0] = -1203.523135984145; result[215, 1] = -873.8499776998053; result[215, 2] = -334.5854298187874;
			result[216, 0] = -1202.1828286242944; result[216, 1] = -844.3128511250956; result[216, 2] = -111.05930079535206;
			result[217, 0] = -1201.4221356445448; result[217, 1] = -815.2170123204448; result[217, 2] = 110.65064579998287;
			result[218, 0] = -1201.3163889055534; result[218, 1] = -785.8223507647646; result[218, 2] = 332.29604560222657;
			result[219, 0] = -1201.86901433943; result[219, 1] = -755.5018366552231; result[219, 2] = 555.7900147614678;
			result[220, 0] = -1233.3712920433081; result[220, 1] = -872.6419775886748; result[220, 2] = -187.1601944467686;
			result[221, 0] = -1232.3072111738368; result[221, 1] = -843.4110308513818; result[221, 2] = 38.27675918735277;
			result[222, 0] = -1231.7699931221046; result[222, 1] = -814.8358138092048; result[222, 2] = 261.66675011907967;
			result[223, 0] = -1231.916939038852; result[223, 1] = -785.8005061929861; result[223, 2] = 484.8665344742888;
			result[224, 0] = -1232.7677008031312; result[224, 1] = -755.413800940861; result[224, 2] = 709.6794652194128;
			result[225, 0] = -1263.0173199469552; result[225, 1] = -871.3793043181485; result[225, 2] = -41.152650518738604;
			result[226, 0] = -1262.2908067391697; result[226, 1] = -842.7032470356567; result[226, 2] = 187.02509621489898;
			result[227, 0] = -1262.0106276101822; result[227, 1] = -814.386967198193; result[227, 2] = 413.45323689411487;
			result[228, 0] = -1262.3890912008862; result[228, 1] = -784.9465690337673; result[228, 2] = 639.8567908849849;
			result[229, 0] = -1263.9628385783167; result[229, 1] = -754.3818705697016; result[229, 2] = 866.806800482139;
			result[230, 0] = -1291.4629907476142; result[230, 1] = -869.8719460783839; result[230, 2] = 105.38749075410357;
			result[231, 0] = -1290.913611335645; result[231, 1] = -842.2305427697892; result[231, 2] = 336.7343436167181;
			result[232, 0] = -1290.5468922996617; result[232, 1] = -813.9172174042318; result[232, 2] = 567.7921343707743;
			result[233, 0] = -1291.3968977781062; result[233, 1] = -783.0964466520851; result[233, 2] = 800.1626404501585;
			result[234, 0] = -1294.1380523244854; result[234, 1] = -750.2184405674698; result[234, 2] = 1033.5845667215383;
			result[235, 0] = -1318.0442019292966; result[235, 1] = -868.6673490027099; result[235, 2] = 254.11152280356654;
			result[236, 0] = -1317.1609941865254; result[236, 1] = -842.2599138495204; result[236, 2] = 488.4914286063462;
			result[237, 0] = -1315.5966077208648; result[237, 1] = -814.66443806261; result[237, 2] = 724.9340075637762;
			result[238, 0] = -1314.9430786301252; result[238, 1] = -782.2624995143696; result[238, 2] = 968.4204251428021;
			result[239, 0] = -1321.1495243353124; result[239, 1] = -751.466537077772; result[239, 2] = 1212.036180271913;
			result[240, 0] = -1342.9812864867222; result[240, 1] = -867.9994707645498; result[240, 2] = 407.5966689113463;
			result[241, 0] = -1342.2273655513197; result[241, 1] = -843.314344440923; result[241, 2] = 642.6427788911732;
			result[242, 0] = -1338.453518393455; result[242, 1] = -817.2892306802516; result[242, 2] = 883.6819901840354;
			result[243, 0] = -1339.831202815678; result[243, 1] = -788.4347338628578; result[243, 2] = 1135.8188767603;
			result[244, 0] = -1351.5583306148103; result[244, 1] = -763.1652922876309; result[244, 2] = 1379.2713616998744;
			result[245, 0] = -1398.046821030171; result[245, 1] = -1029.4941951238648; result[245, 2] = -489.0757465303704;
			result[246, 0] = -1397.6689173796942; result[246, 1] = -998.968579960984; result[246, 2] = -265.26330068036947;
			result[247, 0] = -1397.3401436994575; result[247, 1] = -969.3264090691325; result[247, 2] = -42.77524981549544;
			result[248, 0] = -1396.8402906645788; result[248, 1] = -939.6381206044736; result[248, 2] = 179.59218139824077;
			result[249, 0] = -1396.5776233166794; result[249, 1] = -909.1985121057845; result[249, 2] = 403.49749792668405;
			result[250, 0] = -1429.0305372293128; result[250, 1] = -1029.3642599727364; result[250, 2] = -337.8775746296373;
			result[251, 0] = -1428.3659287498476; result[251, 1] = -998.7272829826442; result[251, 2] = -113.38729773316587;
			result[252, 0] = -1427.8319861592; result[252, 1] = -969.4755461972404; result[252, 2] = 109.74984526982254;
			result[253, 0] = -1427.446568803795; result[253, 1] = -940.0858776710155; result[253, 2] = 332.76671534701677;
			result[254, 0] = -1427.3381702428007; result[254, 1] = -909.4627152219647; result[254, 2] = 557.0845025023897;
			result[255, 0] = -1461.1784881224553; result[255, 1] = -1029.6371357707333; result[255, 2] = -189.9195942644658;
			result[256, 0] = -1460.5740056873833; result[256, 1] = -998.8290085406178; result[256, 2] = 36.57006980009705;
			result[257, 0] = -1460.0260219479592; result[257, 1] = -970.31393015184; result[257, 2] = 261.27950291626377;
			result[258, 0] = -1459.3832529165513; result[258, 1] = -941.3357808676094; result[258, 2] = 485.7531039731349;
			result[259, 0] = -1459.1345244105948; result[259, 1] = -910.1394876039524; result[259, 2] = 711.0526484747992;
			result[260, 0] = -1494.0904519193439; result[260, 1] = -1029.4993184889104; result[260, 2] = -43.64781132709869;
			result[261, 0] = -1493.8460751964928; result[261, 1] = -999.277791854101; result[261, 2] = 185.97424616600355;
			result[262, 0] = -1493.3722768025814; result[262, 1] = -971.8356445643496; result[262, 2] = 413.5093874988979;
			result[263, 0] = -1492.7882951130218; result[263, 1] = -943.5566825446875; result[263, 2] = 640.5514377210246;
			result[264, 0] = -1492.2620117426652; result[264, 1] = -911.0591727112604; result[264, 2] = 869.0439483924207;
			result[265, 0] = -1526.2469121815925; result[265, 1] = -1028.3778540713956; result[265, 2] = 103.02929260984422;
			result[266, 0] = -1526.44612310151; result[266, 1] = -999.4908593139331; result[266, 2] = 336.2752069925747;
			result[267, 0] = -1527.0895153415336; result[267, 1] = -973.7616421589253; result[267, 2] = 568.8032474324733;
			result[268, 0] = -1526.8003034796536; result[268, 1] = -944.8077389173399; result[268, 2] = 802.1404267645231;
			result[269, 0] = -1528.501210301288; result[269, 1] = -911.4517704047898; result[269, 2] = 1033.2384973888113;
			result[270, 0] = -1555.7125608149342; result[270, 1] = -1026.5386941328734; result[270, 2] = 252.28359235313823;
			result[271, 0] = -1556.3011337327384; result[271, 1] = -999.4981086268659; result[271, 2] = 488.4216535644294;
			result[272, 0] = -1556.1460278643028; result[272, 1] = -974.3771511512546; result[272, 2] = 726.8039579097897;
			result[273, 0] = -1557.901462566557; result[273, 1] = -947.4695211881374; result[273, 2] = 977.2651126178608;
			result[274, 0] = -1563.2739298853635; result[274, 1] = -892.186062185834; result[274, 2] = 1240.3638397942948;
			result[275, 0] = -1582.5632053268089; result[275, 1] = -1025.4703648876532; result[275, 2] = 405.46891627378574;
			result[276, 0] = -1582.0290476606403; result[276, 1] = -999.2976311444345; result[276, 2] = 643.7818660310684;
			result[277, 0] = -1580.8027183706101; result[277, 1] = -976.6455423168995; result[277, 2] = 883.0826100657547;
			result[278, 0] = -1560.3722006390672; result[278, 1] = -953.334419053854; result[278, 2] = 1164.310712482019;
			result[279, 0] = -1578.8693840298483; result[279, 1] = -911.1389309818509; result[279, 2] = 1483.2270800764193;
			result[280, 0] = -1622.319984766827; result[280, 1] = -1181.7291175738235; result[280, 2] = -489.6663625441447;
			result[281, 0] = -1622.2156959485083; result[281, 1] = -1151.2519349632814; result[281, 2] = -265.8301406468163;
			result[282, 0] = -1621.9398336555837; result[282, 1] = -1122.4016308975104; result[282, 2] = -43.08410233621198;
			result[283, 0] = -1621.5454130038434; result[283, 1] = -1093.530966998705; result[283, 2] = 179.6020774007056;
			result[284, 0] = -1621.3161969671442; result[284, 1] = -1063.3927863311947; result[284, 2] = 403.54568298412363;
			result[285, 0] = -1654.0647533857898; result[285, 1] = -1181.740680016463; result[285, 2] = -338.71858200564236;
			result[286, 0] = -1653.6140670885459; result[286, 1] = -1151.0812657996435; result[286, 2] = -114.15342136661921;
			result[287, 0] = -1652.9704214847795; result[287, 1] = -1122.7993112711151; result[287, 2] = 109.57732329502946;
			result[288, 0] = -1652.4909009983971; result[288, 1] = -1094.296387400554; result[288, 2] = 333.179464914543;
			result[289, 0] = -1652.1646182018465; result[289, 1] = -1063.8905552041033; result[289, 2] = 557.4525202088945;
			result[290, 0] = -1688.5667434263376; result[290, 1] = -1182.0372167681446; result[290, 2] = -190.53779849505418;
			result[291, 0] = -1688.0446666107662; result[291, 1] = -1151.3067976747343; result[291, 2] = 36.16799403124678;
			result[292, 0] = -1687.1398269022905; result[292, 1] = -1124.5804207266583; result[292, 2] = 261.70037838428914;
			result[293, 0] = -1685.933053786865; result[293, 1] = -1097.1938972718588; result[293, 2] = 486.7549346704419;
			result[294, 0] = -1684.6435171676378; result[294, 1] = -1066.3027038485436; result[294, 2] = 712.2072708638335;
			result[295, 0] = -1725.3137236189839; result[295, 1] = -1182.4037412332796; result[295, 2] = -44.27972281534676;
			result[296, 0] = -1725.095586999598; result[296, 1] = -1152.7135005489868; result[296, 2] = 186.05364745377602;
			result[297, 0] = -1724.2721692759915; result[297, 1] = -1129.0548771783456; result[297, 2] = 414.5811931015931;
			result[298, 0] = -1722.4369283020676; result[298, 1] = -1104.209502663555; result[298, 2] = 642.9417703538221;
			result[299, 0] = -1719.8833629806365; result[299, 1] = -1072.417574967617; result[299, 2] = 869.6069958822934;
			result[300, 0] = -1761.7952051670775; result[300, 1] = -1182.2243608557085; result[300, 2] = 102.3561383312616;
			result[301, 0] = -1762.8962281622148; result[301, 1] = -1154.3351738606527; result[301, 2] = 336.7665003546309;
			result[302, 0] = -1766.060963738986; result[302, 1] = -1135.7043013931873; result[302, 2] = 571.4213476519601;
			result[303, 0] = -1764.8098108426495; result[303, 1] = -1119.3808769053505; result[303, 2] = 803.8511041470321;
			result[304, 0] = -1758.6459434062556; result[304, 1] = -1086.501617122351; result[304, 2] = 1042.4210659947937;
			result[305, 0] = -1794.6492467726655; result[305, 1] = -1181.3058562846124; result[305, 2] = 251.61103553914467;
			result[306, 0] = -1797.1201398179242; result[306, 1] = -1154.8699196150744; result[306, 2] = 489.8156601385115;
			result[307, 0] = -1808.3973729189117; result[307, 1] = -1136.711318874701; result[307, 2] = 728.1207423221688;
			result[308, 0] = -1829.0270839707855; result[308, 1] = -1141.9273500018705; result[308, 2] = 996.7704643908087;
			result[309, 0] = -1826.1437006810518; result[309, 1] = -1121.4555300572365; result[309, 2] = 1213.6702153840877;
			result[310, 0] = -1822.8053790886195; result[310, 1] = -1180.3800001950283; result[310, 2] = 404.9493180916093;
			result[311, 0] = -1824.4732734655358; result[311, 1] = -1154.0991539769466; result[311, 2] = 642.9847435410618;
			result[312, 0] = -1834.2626968094442; result[312, 1] = -1131.8603126604207; result[312, 2] = 891.6340067631387;
			result[313, 0] = -1867.0999875569455; result[313, 1] = -1139.7527343766856; result[313, 2] = 1137.5972602750649;
			result[314, 0] = -1950.1928617927392; result[314, 1] = -1205.0860441756715; result[314, 2] = 1833.4303677525677;
			#endregion

			return result;
		}

		public static Dictionary<int, int> GetSubdomainsOfElements()
		{
			var elementsToSubdomains = new Dictionary<int, int>();
			#region long list of element -> subdomain associations
			elementsToSubdomains[0] = 0;
			elementsToSubdomains[1] = 0;
			elementsToSubdomains[2] = 1;
			elementsToSubdomains[3] = 1;
			elementsToSubdomains[4] = 0;
			elementsToSubdomains[5] = 0;
			elementsToSubdomains[6] = 1;
			elementsToSubdomains[7] = 1;
			elementsToSubdomains[8] = 2;
			elementsToSubdomains[9] = 2;
			elementsToSubdomains[10] = 3;
			elementsToSubdomains[11] = 3;
			elementsToSubdomains[12] = 2;
			elementsToSubdomains[13] = 2;
			elementsToSubdomains[14] = 3;
			elementsToSubdomains[15] = 3;
			elementsToSubdomains[16] = 4;
			elementsToSubdomains[17] = 4;
			elementsToSubdomains[18] = 5;
			elementsToSubdomains[19] = 5;
			elementsToSubdomains[20] = 4;
			elementsToSubdomains[21] = 4;
			elementsToSubdomains[22] = 5;
			elementsToSubdomains[23] = 5;
			elementsToSubdomains[24] = 0;
			elementsToSubdomains[25] = 0;
			elementsToSubdomains[26] = 1;
			elementsToSubdomains[27] = 1;
			elementsToSubdomains[28] = 0;
			elementsToSubdomains[29] = 0;
			elementsToSubdomains[30] = 1;
			elementsToSubdomains[31] = 1;
			elementsToSubdomains[32] = 2;
			elementsToSubdomains[33] = 2;
			elementsToSubdomains[34] = 3;
			elementsToSubdomains[35] = 3;
			elementsToSubdomains[36] = 2;
			elementsToSubdomains[37] = 2;
			elementsToSubdomains[38] = 3;
			elementsToSubdomains[39] = 3;
			elementsToSubdomains[40] = 4;
			elementsToSubdomains[41] = 4;
			elementsToSubdomains[42] = 5;
			elementsToSubdomains[43] = 5;
			elementsToSubdomains[44] = 4;
			elementsToSubdomains[45] = 4;
			elementsToSubdomains[46] = 5;
			elementsToSubdomains[47] = 5;
			elementsToSubdomains[48] = 6;
			elementsToSubdomains[49] = 6;
			elementsToSubdomains[50] = 7;
			elementsToSubdomains[51] = 7;
			elementsToSubdomains[52] = 6;
			elementsToSubdomains[53] = 6;
			elementsToSubdomains[54] = 7;
			elementsToSubdomains[55] = 7;
			elementsToSubdomains[56] = 8;
			elementsToSubdomains[57] = 8;
			elementsToSubdomains[58] = 9;
			elementsToSubdomains[59] = 9;
			elementsToSubdomains[60] = 8;
			elementsToSubdomains[61] = 8;
			elementsToSubdomains[62] = 9;
			elementsToSubdomains[63] = 9;
			elementsToSubdomains[64] = 10;
			elementsToSubdomains[65] = 10;
			elementsToSubdomains[66] = 11;
			elementsToSubdomains[67] = 11;
			elementsToSubdomains[68] = 10;
			elementsToSubdomains[69] = 10;
			elementsToSubdomains[70] = 11;
			elementsToSubdomains[71] = 11;
			elementsToSubdomains[72] = 6;
			elementsToSubdomains[73] = 6;
			elementsToSubdomains[74] = 7;
			elementsToSubdomains[75] = 7;
			elementsToSubdomains[76] = 6;
			elementsToSubdomains[77] = 6;
			elementsToSubdomains[78] = 7;
			elementsToSubdomains[79] = 7;
			elementsToSubdomains[80] = 8;
			elementsToSubdomains[81] = 8;
			elementsToSubdomains[82] = 9;
			elementsToSubdomains[83] = 9;
			elementsToSubdomains[84] = 8;
			elementsToSubdomains[85] = 8;
			elementsToSubdomains[86] = 9;
			elementsToSubdomains[87] = 9;
			elementsToSubdomains[88] = 10;
			elementsToSubdomains[89] = 10;
			elementsToSubdomains[90] = 11;
			elementsToSubdomains[91] = 11;
			elementsToSubdomains[92] = 10;
			elementsToSubdomains[93] = 10;
			elementsToSubdomains[94] = 11;
			elementsToSubdomains[95] = 11;
			elementsToSubdomains[96] = 12;
			elementsToSubdomains[97] = 12;
			elementsToSubdomains[98] = 13;
			elementsToSubdomains[99] = 13;
			elementsToSubdomains[100] = 12;
			elementsToSubdomains[101] = 12;
			elementsToSubdomains[102] = 13;
			elementsToSubdomains[103] = 13;
			elementsToSubdomains[104] = 14;
			elementsToSubdomains[105] = 14;
			elementsToSubdomains[106] = 15;
			elementsToSubdomains[107] = 15;
			elementsToSubdomains[108] = 14;
			elementsToSubdomains[109] = 14;
			elementsToSubdomains[110] = 15;
			elementsToSubdomains[111] = 15;
			elementsToSubdomains[112] = 16;
			elementsToSubdomains[113] = 16;
			elementsToSubdomains[114] = 17;
			elementsToSubdomains[115] = 17;
			elementsToSubdomains[116] = 16;
			elementsToSubdomains[117] = 16;
			elementsToSubdomains[118] = 17;
			elementsToSubdomains[119] = 17;
			elementsToSubdomains[120] = 12;
			elementsToSubdomains[121] = 12;
			elementsToSubdomains[122] = 13;
			elementsToSubdomains[123] = 13;
			elementsToSubdomains[124] = 12;
			elementsToSubdomains[125] = 12;
			elementsToSubdomains[126] = 13;
			elementsToSubdomains[127] = 13;
			elementsToSubdomains[128] = 14;
			elementsToSubdomains[129] = 14;
			elementsToSubdomains[130] = 15;
			elementsToSubdomains[131] = 15;
			elementsToSubdomains[132] = 14;
			elementsToSubdomains[133] = 14;
			elementsToSubdomains[134] = 15;
			elementsToSubdomains[135] = 15;
			elementsToSubdomains[136] = 16;
			elementsToSubdomains[137] = 16;
			elementsToSubdomains[138] = 17;
			elementsToSubdomains[139] = 17;
			elementsToSubdomains[140] = 16;
			elementsToSubdomains[141] = 16;
			elementsToSubdomains[142] = 17;
			elementsToSubdomains[143] = 17;
			elementsToSubdomains[144] = 18;
			elementsToSubdomains[145] = 18;
			elementsToSubdomains[146] = 19;
			elementsToSubdomains[147] = 19;
			elementsToSubdomains[148] = 18;
			elementsToSubdomains[149] = 18;
			elementsToSubdomains[150] = 19;
			elementsToSubdomains[151] = 19;
			elementsToSubdomains[152] = 20;
			elementsToSubdomains[153] = 20;
			elementsToSubdomains[154] = 21;
			elementsToSubdomains[155] = 21;
			elementsToSubdomains[156] = 20;
			elementsToSubdomains[157] = 20;
			elementsToSubdomains[158] = 21;
			elementsToSubdomains[159] = 21;
			elementsToSubdomains[160] = 22;
			elementsToSubdomains[161] = 22;
			elementsToSubdomains[162] = 23;
			elementsToSubdomains[163] = 23;
			elementsToSubdomains[164] = 22;
			elementsToSubdomains[165] = 22;
			elementsToSubdomains[166] = 23;
			elementsToSubdomains[167] = 23;
			elementsToSubdomains[168] = 18;
			elementsToSubdomains[169] = 18;
			elementsToSubdomains[170] = 19;
			elementsToSubdomains[171] = 19;
			elementsToSubdomains[172] = 18;
			elementsToSubdomains[173] = 18;
			elementsToSubdomains[174] = 19;
			elementsToSubdomains[175] = 19;
			elementsToSubdomains[176] = 20;
			elementsToSubdomains[177] = 20;
			elementsToSubdomains[178] = 21;
			elementsToSubdomains[179] = 21;
			elementsToSubdomains[180] = 20;
			elementsToSubdomains[181] = 20;
			elementsToSubdomains[182] = 21;
			elementsToSubdomains[183] = 21;
			elementsToSubdomains[184] = 22;
			elementsToSubdomains[185] = 22;
			elementsToSubdomains[186] = 23;
			elementsToSubdomains[187] = 23;
			elementsToSubdomains[188] = 22;
			elementsToSubdomains[189] = 22;
			elementsToSubdomains[190] = 23;
			elementsToSubdomains[191] = 23;
			#endregion

			return elementsToSubdomains;
		}

		public static Dictionary<int, int> GetSubdomainClusters()
		{
			var result = new Dictionary<int, int>();
			result[0] = 0;
			result[1] = 1;
			result[2] = 0;
			result[3] = 1;
			result[4] = 0;
			result[5] = 1;
			result[6] = 0;
			result[7] = 1;
			result[8] = 0;
			result[9] = 1;
			result[10] = 0;
			result[11] = 1;
			result[12] = 2;
			result[13] = 3;
			result[14] = 2;
			result[15] = 3;
			result[16] = 2;
			result[17] = 3;
			result[18] = 2;
			result[19] = 3;
			result[20] = 2;
			result[21] = 3;
			result[22] = 2;
			result[23] = 3;

			return result;
		}

		public static Dictionary<int, int[]> GetSubdomainNeighbors()
		{
			var result = new Dictionary<int, int[]>();
			result[0] = new int[] { 1, 2, 3, 6, 7, 8, 9, };
			result[1] = new int[] { 0, 2, 3, 6, 7, 8, 9, };
			result[2] = new int[] { 0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, };
			result[3] = new int[] { 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, };
			result[4] = new int[] { 2, 3, 5, 8, 9, 10, 11, };
			result[5] = new int[] { 2, 3, 4, 8, 9, 10, 11, };

			result[6] = new int[] { 0, 1, 2, 3, 7, 8, 9, 12, 13, 14, 15, };
			result[7] = new int[] { 0, 1, 2, 3, 6, 8, 9, 12, 13, 14, 15, };
			result[8] = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 16, 17, };
			result[9] = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, };
			result[10] = new int[] { 2, 3, 4, 5, 8, 9, 11, 14, 15, 16, 17, };
			result[11] = new int[] { 2, 3, 4, 5, 8, 9, 10, 14, 15, 16, 17, };

			result[12] = new int[] { 6, 7, 8, 9, 13, 14, 15, 18, 19, 20, 21, };
			result[13] = new int[] { 6, 7, 8, 9, 12, 14, 15, 18, 19, 20, 21, };
			result[14] = new int[] { 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 21, 22, 23, };
			result[15] = new int[] { 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 19, 20, 21, 22, 23, };
			result[16] = new int[] { 8, 9, 10, 11, 14, 15, 17, 20, 21, 22, 23, };
			result[17] = new int[] { 8, 9, 10, 11, 14, 15, 16, 20, 21, 22, 23, };

			result[18] = new int[] { 12, 13, 14, 15, 19, 20, 21, };
			result[19] = new int[] { 12, 13, 14, 15, 18, 20, 21, };
			result[20] = new int[] { 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, };
			result[21] = new int[] { 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, };
			result[22] = new int[] { 14, 15, 16, 17, 20, 21, 23, };
			result[23] = new int[] { 14, 15, 16, 17, 20, 21, 22, };

			return result;
		}
	}
}
