using System;
using System.Collections.Generic;
using System.Drawing;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using System.Windows.Forms;
using OpenTK.Graphics;

namespace FeaturePointsDescriptors
{
	public class Visualization : Form
	{
		private GLControl glControl;
		private Timer timer;
		private bool idle;
		private double angle;
		private double xMovement;
		private double yMovement;
		private List<ColorPoint3D> points;
		private List<List<ColorPoint3D>> pointsList;
		private int counter = 0;

		private bool pairVizualization;
		private List<DescriptorMatch> matches;
		Color[] colors = { Color.Red, Color.Green, Color.Blue, Color.Cyan, Color.Magenta, Color.Yellow };

		public Visualization (int width, int height, List<List<ColorPoint3D>> pointsList, List<DescriptorMatch> matches, bool pairVizualization)
		{
			this.Text = "Visualization";
			this.Width = width;
			this.Height = height;

			//vytvorenie kontrolky
			glControl = new GLControl ();
			glControl.Location = new Point (10, 10);
			glControl.Size = new Size (95 * width / 100, 95 * height / 100);
			Controls.Add (glControl);

			//pridanie eventov
			glControl.Load += glControl_Load;
			glControl.Paint += glControl_Paint;
			glControl.KeyDown += glControl_KeyDown; //reakcia na stlacanie klavesov

			//automaticke prekreslenie pomocou timera (25FPS)
			timer = new Timer ();
			timer.Interval = 40;
			timer.Tick += timer_onTick;

			//existuje event, ze ak formular nic nerobi, tak generuje event, ze nic nerobi :D vie sa to zijst
			Application.Idle += (sender, e) => idle = true;

			this.pointsList = pointsList;
			this.points = pointsList [0];

			this.matches = matches;
			this.pairVizualization = pairVizualization;
		}
		//automaticke prekreslenie
		void timer_onTick (object sender, EventArgs e)
		{
			glControl_Paint (null, null);
		}
		//priprava gl kontrolky
		void glControl_Load (object sender, EventArgs e)
		{
			GL.ClearColor (Color.Black);
			GL.Viewport (0, 0, this.Width / 2, this.Height);
			GL.Viewport (this.Width / 2, 0, this.Width / 2, this.Height);
			GL.MatrixMode (MatrixMode.Projection);
			GL.LoadIdentity ();
			Matrix4 perspective = Matrix4.CreatePerspectiveFieldOfView (MathHelper.PiOver4, this.Width / this.Height, 0.1f, 100.0f);
			GL.LoadMatrix (ref perspective);
			GL.MatrixMode (MatrixMode.Modelview);
			GL.LoadIdentity ();

			angle = 180;
			yMovement = 3.5;

			timer.Start ();
		}
		//prekreslenie kontrolky
		void glControl_Paint (object sender, PaintEventArgs e)
		{
			if (!idle)
				return;

			idle = false;
			GL.Clear (ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

			for (int i = 0; i < pointsList.Count; i++) {
				GL.Viewport (i * this.Width / pointsList.Count, 0, this.Width / pointsList.Count, this.Height);
				points = pointsList [i];
				draw ();

				if (pairVizualization) {
					// Begin - comparison
					GL.PointSize (10);
					GL.Begin (PrimitiveType.Points);
					for (int j = 6 * counter; j < 6 * counter + 6; j++) {
						ColorPoint3D point;
						double[] descriptor;
						if (i == 0) {
							point = matches [j].Point1.Point;
							descriptor = matches [j].Point1.DoubleDescriptor;
						} else {
							point = matches [j].Point2.Point;				
							descriptor = matches [j].Point2.DoubleDescriptor;
						}
						GL.Color3 (colors [j % 6]);
						GL.Vertex3 (point.X, point.Y, point.Z);
					}
					GL.End ();
					// End - comparison
				}
			}

			glControl.SwapBuffers ();
		}
		//ovladanie klavesami
		void glControl_KeyDown (object sender, KeyEventArgs e)
		{	
			if (e.KeyData == Keys.W) {
				xMovement += -0.1 * Math.Sin (angle / 180 * Math.PI);
				yMovement += 0.1 * Math.Cos (angle / 180 * Math.PI);
			}
			if (e.KeyData == Keys.S) {
				xMovement += 0.1 * Math.Sin (angle / 180 * Math.PI);
				yMovement += -0.1 * Math.Cos (angle / 180 * Math.PI);
			}
			if (e.KeyData == Keys.A) {
				angle -= 0.5;
				angle %= 360;
			}
			if (e.KeyData == Keys.D) {
				angle += 0.5;
				angle %= 360;
			}
			if (e.KeyData == Keys.O) {
				counter++;
				Console.WriteLine (counter);
			}
		}
		//metoda kreslenia
		void draw ()
		{
			// rotacia a posun
			GL.LoadIdentity ();
			GL.Rotate (angle, 0, 1, 0);
			GL.Translate (xMovement, 0, yMovement);

			// kreslenie bodov
			GL.PointSize (3);
			GL.Begin (PrimitiveType.Points);			

			int density = 1;
			for (int i = 0; i < points.Count; i += density) {
				Color c = Color.FromArgb (points [i].R, points [i].G, points [i].B); //points nazvem nejaky list a obsahuje datovu strukturu, ktora udrziava X, Y, Z, R, G, B
				GL.Color3 (c);
				GL.Vertex3 (points [i].X, points [i].Y, points [i].Z);
			}

			GL.End ();
		}
	}
}

