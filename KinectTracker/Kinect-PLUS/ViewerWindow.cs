using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;

namespace KinectTracker
{//Ventana de visualización de las imágenes IR de las dos cámaras, lado a lado
     //BuildForm(): Crea la ventana y el PictureBox, y se encarga de pintar el bitmap combinado.
     //ShowWindow(): Lanza el bucle de mensajes de la ventana. UpdateIRImages() actualiza las dos imágenes IR a la vez.
     //Close(): Cierra la ventana de forma segura desde cualquier hilo.
     //UpdateIRImages(): Actualiza las dos imágenes IR a la vez (A a la izquierda, B a la derecha) y fuerza el repintado del PictureBox.
     //EscribirMitad(): Escribe un frame IR (640x480) en una mitad del bitmap combinado, con offset horizontal.
     //ConsumirCaptura(): Devuelve true si el usuario ha pulsado ESPACIO para capturar un frame .

    public class ViewerWindow
    {
        //Construcción de ventana y controles
        private PictureBox irPictureBox;   //único panel: A | B lado a lado
        private Form viewerForm;

        //Doble buffer de la imagen combinada (ancho doble)
        private Bitmap comboBuffer0 = null;
        private Bitmap comboBuffer1 = null;
        private bool useCombo0 = true;
        private Bitmap currentCombo = null;

        private const int W = Constants.IMG_WIDTH;
        private const int H = Constants.IMG_HEIGHT;

        private volatile bool capturaSolicitada = false;

        public ViewerWindow()
        {
            //Bitmap combinado: dos cámaras una al lado de la otra
            comboBuffer0 = new Bitmap(W * 2, H, PixelFormat.Format32bppRgb);
            comboBuffer1 = new Bitmap(W * 2, H, PixelFormat.Format32bppRgb);
            BuildForm();
        }

        private void BuildForm()
        {
            Application.EnableVisualStyles();

            viewerForm = new Form();
            viewerForm.Text = "Estéreo IR - A | B";
            viewerForm.Size = new Size(1320, 560);
            viewerForm.StartPosition = FormStartPosition.CenterScreen;

            viewerForm.KeyPreview = true;
            viewerForm.KeyDown += (s, e) => { if (e.KeyCode == Keys.Space) capturaSolicitada = true; };

            //Un solo PictureBox que ocupa toda la ventana
            irPictureBox = new PictureBox();
            irPictureBox.Dock = DockStyle.Fill;
            irPictureBox.BackColor = Color.Black;
            viewerForm.Controls.Add(irPictureBox);

            //Pintado manual del bitmap combinado
            irPictureBox.Paint += (s, e) =>
            {
                if (currentCombo != null)
                    e.Graphics.DrawImage(currentCombo, irPictureBox.ClientRectangle);
            };

            var h = irPictureBox.Handle;   //fuerza la creación del handle
        }

        public void ShowWindow()
        {
            Application.Run(viewerForm);
        }

        //Cierra la ventana de forma segura desde cualquier hilo (p.ej. al terminar
        //la captura de profiling). Cerrar el form termina Application.Run y dispara
        //el flujo de cierre normal (Stop -> Summary).
        public void Close()
        {
            if (viewerForm == null) return;
            if (viewerForm.IsHandleCreated && viewerForm.InvokeRequired)
                viewerForm.BeginInvoke((Action)(() => viewerForm.Close()));
            else
                viewerForm.Close();
        }

        //Actualizar las dos imágenes IR a la vez (A a la izquierda, B a la derecha)
        public void UpdateIRImages(byte[] irA, byte[] irB)
        {
            if (viewerForm == null || !viewerForm.IsHandleCreated) return;

            Bitmap combo = useCombo0 ? comboBuffer0 : comboBuffer1;
            useCombo0 = !useCombo0;

            //Volcar A en la mitad izquierda y B en la derecha del mismo bitmap
            EscribirMitad(combo, irA, 0);   //x offset 0
            EscribirMitad(combo, irB, W);   //x offset W

            currentCombo = combo;

            try
            {
                viewerForm.BeginInvoke((MethodInvoker)delegate
                {
                    irPictureBox.Invalidate();
                });
            }
            catch { }
        }

        //Escribir un frame IR (640x480) en una mitad del bitmap combinado, con offset horizontal
        private void EscribirMitad(Bitmap combo, byte[] irPixels, int xOffset)
        {
            BitmapData d = combo.LockBits(
                new Rectangle(xOffset, 0, W, H),
                ImageLockMode.WriteOnly,
                combo.PixelFormat);

            //Copia fila a fila: el stride del bitmap combinado es el doble de ancho
            int srcStride = W * 4;
            for (int y = 0; y < H; y++)
            {
                IntPtr destRow = d.Scan0 + y * d.Stride;
                System.Runtime.InteropServices.Marshal.Copy(irPixels, y * srcStride, destRow, srcStride);
            }
            combo.UnlockBits(d);
        }

        public bool ConsumirCaptura()
        {
            if (capturaSolicitada) { capturaSolicitada = false; return true; }
            return false;
        }
    }
}