using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;

namespace KinectTracker {
    public class ViewerWindow
    {
        //Construcción de ventana y controles
        private PictureBox irPictureBox;
        private PictureBox depthPictureBox;
        private Form viewerForm;

        // Doble buffer IR
        private Bitmap irBufferA = null;
        private Bitmap irBufferB = null;
        private bool useIrA = true;

        // Doble buffer Depth
        private Bitmap depthBufferA = null;
        private Bitmap depthBufferB = null;
        private bool useDepthA = true;

        public ViewerWindow()
        {
            // Configuración de la ventana
            irBufferA = new Bitmap(Constants.IMG_WIDTH, Constants.IMG_HEIGHT, PixelFormat.Format32bppRgb);
            irBufferB = new Bitmap(Constants.IMG_WIDTH, Constants.IMG_HEIGHT, PixelFormat.Format32bppRgb);

            depthBufferA = new Bitmap(Constants.IMG_WIDTH, Constants.IMG_HEIGHT, PixelFormat.Format32bppRgb);
            depthBufferB = new Bitmap(Constants.IMG_WIDTH, Constants.IMG_HEIGHT, PixelFormat.Format32bppRgb);

            BuildForm();
        }

        private void BuildForm()
        {
            Application.EnableVisualStyles();

            //Crea ventana
            viewerForm = new Form();
            viewerForm.Text = "Kinect IR + Depth Stream";
            viewerForm.Size = new Size(1320, 560);
            viewerForm.StartPosition = FormStartPosition.CenterScreen;

            TableLayoutPanel layout = new TableLayoutPanel();
            layout.Dock = DockStyle.Fill;
            layout.ColumnCount = 2;
            layout.RowCount = 2;
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
            layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30)); //Labels arriba
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); //Imagenes abajo
            viewerForm.Controls.Add(layout);

            //Labels descriptivos
            Label irLabel = new Label();
            irLabel.Text = "IR Stream (con threshold)";
            irLabel.Dock = DockStyle.Fill;
            irLabel.TextAlign = ContentAlignment.MiddleCenter;
            irLabel.Font = new Font("Consolas", 10, FontStyle.Bold);
            layout.Controls.Add(irLabel, 0, 0);

            Label depthLabel = new Label();
            depthLabel.Text = "Depth Stream (cerca=blanco, lejos=negro)";
            depthLabel.Dock = DockStyle.Fill;
            depthLabel.TextAlign = ContentAlignment.MiddleCenter;
            depthLabel.Font = new Font("Consolas", 10, FontStyle.Bold);
            layout.Controls.Add(depthLabel, 1, 0);

            //Crea PictureBox IR (izquierda)
            irPictureBox = new PictureBox();
            irPictureBox.Dock = DockStyle.Fill;
            irPictureBox.SizeMode = PictureBoxSizeMode.Zoom;
            irPictureBox.BackColor = Color.Black;
            layout.Controls.Add(irPictureBox, 0, 1);

            //Crea PictureBox Depth (derecha)
            depthPictureBox = new PictureBox();
            depthPictureBox.Dock = DockStyle.Fill;
            depthPictureBox.SizeMode = PictureBoxSizeMode.Zoom;
            depthPictureBox.BackColor = Color.Black;
            layout.Controls.Add(depthPictureBox, 1, 1);
        }

        public void ShowWindow()
        {
            Application.Run(viewerForm); //El bloqueo de ventana se maneja aquí, el programa sigue ejecutándose hasta que se cierre la ventana
        }

        public void UpdateIRImage(byte[] irPixels) //Actualizar imagen IR (llamado desde el handler de Kinect)
        {
            if (irPictureBox == null || irPictureBox.IsDisposed || !irPictureBox.IsHandleCreated)
                return;

            Bitmap writeBuffer = useIrA ? irBufferA : irBufferB;

            // Escribir en el buffer IR
            BitmapData bmpData = writeBuffer.LockBits(
                new Rectangle(0, 0, Constants.IMG_WIDTH, Constants.IMG_HEIGHT),
                ImageLockMode.WriteOnly,
                writeBuffer.PixelFormat);

            System.Runtime.InteropServices.Marshal.Copy(irPixels, 0, bmpData.Scan0, irPixels.Length);
            writeBuffer.UnlockBits(bmpData);

            Bitmap displayBuffer = writeBuffer;
            useIrA = !useIrA;

            try
            {
                irPictureBox.Invoke((MethodInvoker)delegate
                {
                    irPictureBox.Image = displayBuffer;
                });
            }
            catch { }
        }

        public void UpdateDepthImage(byte[] depthPixels) //Actualizar imagen Depth (llamado desde el handler de Kinect)
        {
            if (depthPictureBox == null || depthPictureBox.IsDisposed || !depthPictureBox.IsHandleCreated)
                return;
            Bitmap writeDepthBuffer = useDepthA ? depthBufferA : depthBufferB;

            BitmapData bmpDataDepth = writeDepthBuffer.LockBits(
                new Rectangle(0, 0, Constants.IMG_WIDTH, Constants.IMG_HEIGHT),
                ImageLockMode.WriteOnly,
                writeDepthBuffer.PixelFormat);

            System.Runtime.InteropServices.Marshal.Copy(depthPixels, 0, bmpDataDepth.Scan0, depthPixels.Length);
            writeDepthBuffer.UnlockBits(bmpDataDepth);

            Bitmap displayDepthBuffer = writeDepthBuffer;
            useDepthA = !useDepthA;

            try
            {
                depthPictureBox.Invoke((MethodInvoker)delegate
                {
                    depthPictureBox.Image = displayDepthBuffer;
                });
            }
            catch { }
        }




    }
}

