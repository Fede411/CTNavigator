using System;

namespace KinectTracker {
    public static class ImageUtils
    {
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
    }
}

