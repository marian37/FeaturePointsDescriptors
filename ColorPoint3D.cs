using System;

namespace FeaturePointsDescriptors
{
	/**
	 * 
	 * Trieda reprezentujúca farebný bod - súradnice x, y, z a RGB formát farby
	 * 
	 **/
	public class ColorPoint3D
	{
		private readonly float x;
		private readonly float y;
		private readonly float z;
		private int r;
		private int g;
		private int b;
		private int idx;

		public ColorPoint3D (float x, float y, float z, int r, int g, int b)
		{
			this.x = x;
			this.y = y;
			this.z = z;
			this.r = r;
			this.g = g;
			this.b = b;
		}

		public float X {
			get {
				return this.x;
			}
		}

		public float Y {
			get {
				return this.y;
			}
		}

		public float Z {
			get {
				return this.z;
			}
		}

		public int R {
			get {
				return this.r;
			}
			set {
				r = value;
			}
		}

		public int G {
			get {
				return this.g;
			}
			set {
				g = value;
			}
		}

		public int B {
			get {
				return this.b;
			}
			set {
				b = value;
			}
		}

		public int Idx {
			get {
				return this.idx;
			}
			set {
				idx = value;
			}
		}

		public override string ToString ()
		{
			return this.X + " " + this.Y + " " + this.Z + " " + this.R + " " + this.G + " " + this.B; 
		}

		public override bool Equals (object obj)
		{
			if (obj == null) {
				return false;
			}

			ColorPoint3D point = obj as ColorPoint3D;
			if ((System.Object)point == null) {
				return false;
			}

			return (this.X == point.X)
			&& (this.Y == point.Y)
			&& (this.Z == point.Z)
			&& (this.R == point.R)
			&& (this.G == point.G)
			&& (this.B == point.B);
		}

		public override int GetHashCode ()
		{
			int x = (int)this.X;
			int y = (int)this.Y;
			int z = (int)this.Z;
			return x ^ y ^ z ^ r ^ g ^ b;
		}
	}
}

