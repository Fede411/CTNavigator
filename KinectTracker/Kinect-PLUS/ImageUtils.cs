using System;

namespace KinectTracker {
    public static class ImageUtils
    {
        // Convierte el IR crudo (2 bytes/px) a escala de grises BGRA, SIN threshold.
        // Para calibración: MATLAB necesita ver el tablero, no blobs binarizados.
        public static byte[] IRaGris(byte[] colorPixels)
        {
            byte[] gris = new byte[Constants.IMG_WIDTH * Constants.IMG_HEIGHT * 4];
            for (int i = 0; i < Constants.IMG_WIDTH * Constants.IMG_HEIGHT; i++)
            {
                int irValue = colorPixels[i * 2] | (colorPixels[i * 2 + 1] << 8);
                byte intensity = (byte)(irValue >> 8);
                gris[i * 4] = intensity;
                gris[i * 4 + 1] = intensity;
                gris[i * 4 + 2] = intensity;
                gris[i * 4 + 3] = 255;
            }
            return gris;
        }

        public static void GuardarPNG(byte[] irPixels, string path)
        {
            using (var bmp = new System.Drawing.Bitmap(Constants.IMG_WIDTH, Constants.IMG_HEIGHT,
                System.Drawing.Imaging.PixelFormat.Format32bppRgb))
            {
                var d = bmp.LockBits(
                    new System.Drawing.Rectangle(0, 0, Constants.IMG_WIDTH, Constants.IMG_HEIGHT),
                    System.Drawing.Imaging.ImageLockMode.WriteOnly, bmp.PixelFormat);
                System.Runtime.InteropServices.Marshal.Copy(irPixels, 0, d.Scan0, irPixels.Length);
                bmp.UnlockBits(d);
                bmp.Save(path, System.Drawing.Imaging.ImageFormat.Png);
            }
        }

        public static void DrawCircle(byte[] pixels, int cx, int cy, int radius, byte r, byte g, byte b)
        //Dibujar círculo en imagen BGRA en memoria
        {
            //Algoritmo de círculo (puntos del borde)
            for (int angle = 0; angle < 360; angle++)
            {
                double rad = angle * Math.PI / 180.0;
                int x = cx + (int)(radius * Math.Cos(rad));
                int y = cy + (int)(radius * Math.Sin(rad));

                if (x >= 0 && x < Constants.IMG_WIDTH && y >= 0 && y < Constants.IMG_HEIGHT)
                {
                    int idx = (y * Constants.IMG_WIDTH + x) * 4;
                    pixels[idx] = b;     //Blue
                    pixels[idx + 1] = g; //Green
                    pixels[idx + 2] = r; //Red
                                         //pixels[idx + 3] ya es 255
                }
            }
        }

        public static void DrawLine(byte[] pixels, int x0, int y0, int x1, int y1, byte r, byte g, byte b)
        {
            // Algoritmo de Bresenham
            int dx = Math.Abs(x1 - x0);
            int dy = Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;

            while (true)
            {
                if (x0 >= 0 && x0 < Constants.IMG_WIDTH && y0 >= 0 && y0 < Constants.IMG_HEIGHT)
                {
                    int idx = (y0 * Constants.IMG_WIDTH + x0) * 4;
                    pixels[idx] = b;
                    pixels[idx + 1] = g;
                    pixels[idx + 2] = r;
                }

                if (x0 == x1 && y0 == y1) break;

                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx) { err += dx; y0 += sy; }
            }
        }
    }
}

