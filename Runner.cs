using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Globalization;
using Accord.MachineLearning.Structures;
using Accord.Math;
using System.Diagnostics;
using Accord.Statistics.Analysis;

namespace FeaturePointsDescriptors
{
	public class Runner
	{
		private static readonly float VOXEL_SIZE = 0.04f;

		private static readonly float MAX_SHELL_DISTANCE = 0.3f;

		private static readonly float TRESHOLD_SQUARE = 2 * VOXEL_SIZE * VOXEL_SIZE;

		private static readonly int NUMBER_OF_SHELL_BINS = 20;

		private static readonly int NUMBER_OF_COLORS = 20;

		private static readonly int RANSAC_REPEAT = 10000;

		private static readonly int RANSAC_POINTS = 4;

		private static readonly int SUBLIST_COEFF = 1;

		private static readonly int NEIGHBORS = 3;

		private static readonly double COS_TRESHOLD = Math.Cos (15 * Math.PI / 180);

		private static readonly double FEATURE_TRESHOLD = 0.0001;

		private static readonly double COLOR_TRESHOLD = 6;

		private static readonly double COLLINEAR_TRESHOLD = 0.15f;

		private static readonly bool PRINT = false;

		private static readonly bool PAIR_VIZUALIZATION = false;

		public Runner ()
		{		
		}

		private List<ColorPoint3D> readInputFromFile (string fileName)
		{
			string[] rows = System.IO.File.ReadAllLines (fileName);
			string row = rows [1];
			string[] tokens = row.Split (new string[] { " " }, StringSplitOptions.RemoveEmptyEntries);
			int numberOfInputPoints = Int32.Parse (tokens [0]);
			List<ColorPoint3D> input = new List<ColorPoint3D> (numberOfInputPoints);

			CultureInfo ci = CultureInfo.InvariantCulture;
			//CultureInfo ci = CultureInfo.CurrentCulture;

			Object lockObject = new Object ();

			Parallel.For (0, numberOfInputPoints, i => {			
				//for (int i = 0; i < numberOfInputPoints; i++) {
				string[] tokensLocal = rows [i + 2].Split (new string[] { " " }, StringSplitOptions.RemoveEmptyEntries);
				float x = float.Parse (tokensLocal [0], ci);
				float y = float.Parse (tokensLocal [1], ci);
				float z = float.Parse (tokensLocal [2], ci);

				int r = (int)(float.Parse (tokensLocal [3], ci) * 255);
				int g = (int)(float.Parse (tokensLocal [4], ci) * 255);
				int b = (int)(float.Parse (tokensLocal [5], ci) * 255);

				ColorPoint3D point = new ColorPoint3D (x, y, z, r, g, b);
				lock (lockObject) {
					input.Add (point);
				}
				//}
			});
			

			return input;
		}

		static void descriptorsToString (List<PointDescriptor> pointDescriptors)
		{
			foreach (PointDescriptor descriptor in pointDescriptors) {
				double count = 0;
				for (int i = 0; i < descriptor.DoubleDescriptor.Length; i++) {
					if (i == NUMBER_OF_SHELL_BINS) {
						Console.Write ("| " + count + " | ");
					}
					count += descriptor.DoubleDescriptor [i];
					Console.Write (descriptor.DoubleDescriptor [i] + " ");
				}
				Console.WriteLine ();
			}
		}

		private static double[,] prepareData (List<ColorPoint3D> points)
		{
			double[,] data = new double[points.Count, 3];

			for (int i = 0; i < points.Count; i++) {
				data [i, 0] = points [i].X;
				data [i, 1] = points [i].Y;
				data [i, 2] = points [i].Z;
			}

			return data;
		}

		static ColorPoint3D pcaNormalVector (List<ColorPoint3D> points)
		{
			if (points.Count < 7)
				return null;

			double[,] data = prepareData (points);		

			var pca = new PrincipalComponentAnalysis (data, AnalysisMethod.Center);
			pca.Compute ();

			double[,] componentMatrix = pca.ComponentMatrix;

			return new ColorPoint3D ((float)componentMatrix [0, 2], (float)componentMatrix [1, 2], (float)componentMatrix [2, 2], 0, 0, 0);
		}

		static double pcaFeature (List<ColorPoint3D> points)
		{
			if (points.Count < 7)
				return double.MaxValue;

			double[,] data = prepareData (points);		

			var pca = new PrincipalComponentAnalysis (data, AnalysisMethod.Center);
			pca.Compute ();

			double[] eigenvalues = pca.Eigenvalues;

			double eigenvaluesSum = 0;
			double minEigenvalue = double.MaxValue;
			for (int i = 0; i < eigenvalues.Length; i++) {
				if (eigenvalues [i] < minEigenvalue) {
					minEigenvalue = eigenvalues [i];
				}
				eigenvaluesSum += eigenvalues [i];
			}

			return minEigenvalue / eigenvaluesSum;
		}

		public static void Main ()
		{
			Runner runner = new Runner ();
			FeaturePointsDescriptors fpd = new FeaturePointsDescriptors ();
			List<List<ColorPoint3D>> pointClouds = new List<List<ColorPoint3D>> ();

			Stopwatch stopwatch = Stopwatch.StartNew ();
			Stopwatch watch = Stopwatch.StartNew ();

			List<ColorPoint3D> pointCloud1 = runner.readInputFromFile ("../../pc1.off");

			watch.Stop ();
			Console.WriteLine ("First point cloud read from file. " + watch.ElapsedMilliseconds + " ms");
			VoxelGridFilter voxelGridFilter1 = new VoxelGridFilter ();

			watch.Restart ();
			pointCloud1 = voxelGridFilter1.filter (pointCloud1, VOXEL_SIZE);
			watch.Stop ();
			Console.WriteLine ("VoxelGridFilter finished on first point cloud. " + watch.ElapsedMilliseconds + " ms");

			pointClouds.Add (pointCloud1);

			List<List<List<List<ColorPoint3D>>>> grid1 = voxelGridFilter1.getGrid;

			watch.Restart ();
			List<PointDescriptor> descriptors1 = fpd.getDescriptors (grid1, VOXEL_SIZE, MAX_SHELL_DISTANCE, NUMBER_OF_SHELL_BINS, NUMBER_OF_COLORS, pointCloud1.Count);
			watch.Stop ();
			Console.WriteLine ("First point cloud descriptors calculated. " + watch.ElapsedMilliseconds + " ms");

			if (PRINT) {
				descriptorsToString (descriptors1);
			}				

			watch.Restart ();
			List<ColorPoint3D> pointCloud2 = runner.readInputFromFile ("../../pc2.off");
			watch.Stop ();
			Console.WriteLine ("Second point cloud read from file. " + watch.ElapsedMilliseconds + " ms");

			VoxelGridFilter voxelGridFilter2 = new VoxelGridFilter ();

			watch.Restart ();
			pointCloud2 = voxelGridFilter2.filter (pointCloud2, VOXEL_SIZE);
			watch.Stop ();
			Console.WriteLine ("VoxelGridFilter finished on second point cloud. " + watch.ElapsedMilliseconds + " ms");

			pointClouds.Add (pointCloud2);					

			List<List<List<List<ColorPoint3D>>>> grid2 = voxelGridFilter2.getGrid;

			watch.Restart ();
			List<PointDescriptor> descriptors2 = fpd.getDescriptors (grid2, VOXEL_SIZE, MAX_SHELL_DISTANCE, NUMBER_OF_SHELL_BINS, NUMBER_OF_COLORS, pointCloud2.Count);
			watch.Stop ();
			Console.WriteLine ("Second point cloud descriptors calculated. " + watch.ElapsedMilliseconds + " ms");

			if (PRINT) {
				descriptorsToString (descriptors2);	
			}
				
			// PCA - BEGIN
			watch.Restart ();
			List<PointDescriptor> descriptors1new = new List<PointDescriptor> ();
			foreach (PointDescriptor descriptor in descriptors1) {
				ColorPoint3D small = pcaNormalVector (descriptor.SmallNeighbourhood);

				if (small == null)
					continue;

				ColorPoint3D big = pcaNormalVector (descriptor.BigNeighbourhood);

				double cos = Math.Abs (big.X * small.X + big.Y * small.Y + big.Z * small.Z) / (FeaturePointsDescriptors.vectorLength (big) * FeaturePointsDescriptors.vectorLength (small));
				if (cos < COS_TRESHOLD) {
					//descriptor.Point.R = 255;
					//descriptor.Point.G = 0;
					//descriptor.Point.B = 0;
					descriptor.Feature = pcaFeature (descriptor.BigNeighbourhood);
					descriptors1new.Add (descriptor);
				}
			}
			watch.Stop ();
			Console.WriteLine ("First point cloud edge points detected. " + watch.ElapsedMilliseconds + " ms");

			watch.Restart ();
			List<PointDescriptor> descriptors2new = new List<PointDescriptor> ();
			foreach (PointDescriptor descriptor in descriptors2) {
				ColorPoint3D small = pcaNormalVector (descriptor.SmallNeighbourhood);

				if (small == null)
					continue;

				ColorPoint3D big = pcaNormalVector (descriptor.BigNeighbourhood);

				double cos = Math.Abs (big.X * small.X + big.Y * small.Y + big.Z * small.Z) / (FeaturePointsDescriptors.vectorLength (big) * FeaturePointsDescriptors.vectorLength (small));
				if (cos < COS_TRESHOLD) {
					//descriptor.Point.R = 255;
					//descriptor.Point.G = 0;
					//descriptor.Point.B = 0;
					descriptor.Feature = pcaFeature (descriptor.BigNeighbourhood);
					descriptors2new.Add (descriptor);
				}
			}
			watch.Stop ();
			Console.WriteLine ("First point cloud edge points detected. " + watch.ElapsedMilliseconds + " ms");

			descriptors1 = descriptors1new;
			descriptors2 = descriptors2new;
			// PCA - END


			double[][] doubleDescriptors1 = new double[descriptors1.Count][];
			for (int i = 0; i < doubleDescriptors1.Length; i++) {
				doubleDescriptors1 [i] = descriptors1 [i].DoubleDescriptor;
			}

			double[][] doubleDescriptors2 = new double[descriptors2.Count][];
			for (int i = 0; i < doubleDescriptors2.Length; i++) {
				doubleDescriptors2 [i] = descriptors2 [i].DoubleDescriptor;
			}

			Console.WriteLine ("Keypoints: " + descriptors1.Count + " " + descriptors2.Count);
				
			// KD trees
			//KDTree<PointDescriptor> pointCloud1Tree = KDTree.FromData<PointDescriptor> (doubleDescriptors1, descriptors1.ToArray (), true);
			//Console.WriteLine ("Tree build");
			watch.Restart ();

			KDTree<PointDescriptor> pointCloud2Tree = KDTree.FromData<PointDescriptor> (doubleDescriptors2, descriptors2.ToArray (), true);
			Console.WriteLine ("Tree build");
			ConcurrentBag<DescriptorMatch> matches = new ConcurrentBag<DescriptorMatch> ();

			Parallel.ForEach (descriptors1, (descriptor) => {			
				//foreach (PointDescriptor descriptor in descriptors1) {
				/*double distance = -1;
				double distance2 = -1;
				PointDescriptor nearestNeighbor = pointCloud1Tree.ApproximateNearest (descriptor.DoubleDescriptor, 0.5, out distance).Value;
				PointDescriptor nearestNeighbor = pointCloud2Tree.Nearest (descriptor.DoubleDescriptor, out distance).Value;
				*/
						
				KDTreeNodeCollection<PointDescriptor> list = pointCloud2Tree.Nearest (descriptor.DoubleDescriptor, NEIGHBORS);
				foreach (KDTreeNodeDistance<PointDescriptor> node in list) {
					matches.Add (new DescriptorMatch (descriptor, node.Node.Value, node.Distance));
				}
			
				//matches.Add (new DescriptorMatch (descriptor, nearestNeighbor, distance));
				//PointDescriptor nearestToNearestNeighbor = pointCloud1Tree.Nearest (nearestNeighbor.DoubleDescriptor, out distance2).Value;
				/*KDTreeNodeList<PointDescriptor> list = pointCloud2Tree.Nearest (nearestNeighbor.DoubleDescriptor, distance);
				PointDescriptor nearestToNearestNeighbor = list [0].Node.Value;
				distance2 = list [0].Distance;*/
				//if (distance == distance2 && descriptor.Point.Equals (nearestToNearestNeighbor.Point)) {
				//matches.Add (new DescriptorMatch (descriptor, nearestNeighbor, distance));
				//}			
				//}
			});

			// Matchovanie na základe zakrivenia
			/*
			Parallel.ForEach (descriptors1, (descriptor) => {
				foreach (PointDescriptor descriptor2 in descriptors2) {
					double featureDifference = Math.Abs (descriptor.Feature - descriptor2.Feature);					
					if (featureDifference < FEATURE_TRESHOLD) {					
						*/
			/*descriptor.Point.R = 0;
						descriptor.Point.G = 0;
						descriptor.Point.B = 255;
						descriptor2.Point.R = 0;
						descriptor2.Point.G = 0;
						descriptor2.Point.B = 255;
						*/
			/*if (SUBLIST_COEFF > 1) {
							matches.Add (new DescriptorMatch (descriptor, descriptor2, Distance.Euclidean (descriptor.DoubleDescriptor, descriptor2.DoubleDescriptor)));
						} else {
							matches.Add (new DescriptorMatch (descriptor, descriptor2, featureDifference));
						}
					}
				}
			});
			*/

			// Matchovanie rohových podľa farby
			/*Parallel.ForEach (descriptors1, (descriptor) => {
				foreach (PointDescriptor descriptor2 in descriptors2) {		
					if (descriptor.Point.G < 255 && descriptor2.Point.G < 255) {
						matches.Add (new DescriptorMatch (descriptor, descriptor2, 0));
					}
				}
			});
			*/

			// Matchovanie na základe farby
			/*Parallel.ForEach (descriptors1, (descriptor) => {
				foreach (PointDescriptor descriptor2 in descriptors2) {
					ColorPoint3D p1 = descriptor.Point;
					ColorPoint3D p2 = descriptor2.Point;
					double colorDifference = Math.Abs (p1.R - p2.R) + Math.Abs (p1.G - p2.G) + Math.Abs (p1.B - p2.B);
					//Console.WriteLine ("Color: " + colorDifference);
					if (colorDifference < COLOR_TRESHOLD) {
						matches.Add (new DescriptorMatch (descriptor, descriptor2, colorDifference));
					}
				}
			});*/

			// Matchovanie všetkých
			/*Parallel.ForEach (descriptors1, (descriptor) => {
				foreach (PointDescriptor descriptor2 in descriptors2) {
					matches.Add (new DescriptorMatch (descriptor, descriptor2, 0));
				}
			});
			*/
			
			watch.Stop ();
			Console.WriteLine ("Pairs created. " + watch.ElapsedMilliseconds + " ms");

			// brute-force = pomalé
			/*ConcurrentBag<DescriptorMatch> matches = new ConcurrentBag<DescriptorMatch> ();
			Parallel.ForEach (descriptors2, (descriptor) => {
				PointDescriptor bestDescriptor = null;
				int bestSquareDistance = int.MaxValue;
				int squareDistance;
				for (int i = 0; i < descriptors1.Length; i++) {
					squareDistance = fpd.getDescriptorSquareDistance (descriptor, descriptors1 [i]);
					if (squareDistance < bestSquareDistance) {
						bestSquareDistance = squareDistance;
						bestDescriptor = descriptors1 [i];
					}
				}
				if (bestDescriptor != null) {
					for (int i = 0; i < descriptors2.Length; i++) {
						squareDistance = fpd.getDescriptorSquareDistance (bestDescriptor, descriptors2 [i]);
						if (squareDistance < bestSquareDistance) {
							bestDescriptor = null;
							break;
						}
					}
				}
				if (bestDescriptor != null) {
					matches.Add (new DescriptorMatch (descriptor, bestDescriptor, Math.Sqrt (bestSquareDistance)));
				}
			});
			Console.WriteLine ("Done.");
			*/

			List<DescriptorMatch> matchesList = matches.ToList ();
			List<DescriptorMatch> matchesSublist = matchesList;
			if (SUBLIST_COEFF > 1) {
				matchesList.Sort ((x, y) => x.Match.CompareTo (y.Match));
				Console.WriteLine ("Sorted");
			
				if (PRINT) {
					foreach (DescriptorMatch match in matchesList) {
						Console.WriteLine (match.Match + " "
						+ match.Point1.Point.X + " " + match.Point1.Point.Y + " " + match.Point1.Point.Z + " "
						+ match.Point2.Point.X + " " + match.Point2.Point.Y + " " + match.Point2.Point.Z);
					}

					Console.WriteLine ("Min: " + matchesList [0].Match + " Max: " + matchesList [matchesList.Count - 1].Match + " Part: " + matchesList [matchesList.Count / SUBLIST_COEFF].Match);
				}

				matchesSublist = matchesList.GetRange (0, matchesList.Count / SUBLIST_COEFF);

				Console.WriteLine (matchesList.Count + " " + matchesSublist.Count);
			} 
	
			watch.Restart ();
			double[,] coefficients = fpd.ransacTransformation (matchesSublist, RANSAC_REPEAT, TRESHOLD_SQUARE, COLLINEAR_TRESHOLD, RANSAC_POINTS, pointCloud1.Count, pointCloud2.Count);
			watch.Stop ();
			Console.WriteLine ("Ransac ended. " + watch.ElapsedMilliseconds + " ms");

			if (PRINT) {
				for (int i = 0; i < coefficients.GetLength (0); i++) {
					for (int j = 0; j < coefficients.GetLength (1); j++) {
						Console.Write (coefficients [i, j] + " ");
					}
					Console.WriteLine ();
				}
			}

			List<ColorPoint3D> pointCloudResult = fpd.transformPointCloud (pointCloud2, coefficients);

			pointCloudResult.AddRange (pointCloud1);
			if (!PAIR_VIZUALIZATION) {
				pointClouds.Clear ();
				pointClouds.Add (pointCloudResult);
			}			

			stopwatch.Stop ();
			Console.WriteLine ("Elapsed time: " + stopwatch.ElapsedMilliseconds + " ms");
				
			try {				
				Visualization visualization = new Visualization (1800, 900, pointClouds, matchesList, PAIR_VIZUALIZATION);
				visualization.ShowDialog ();
			} catch (Exception e) {
				Console.Write (e.StackTrace);
			}			

		}
	}
}

