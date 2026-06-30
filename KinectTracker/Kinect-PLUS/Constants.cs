namespace KinectTracker
{
    public static class Constants
    {
        public const string KINECT_A_ID = @"USB\VID_045E&PID_02AE\A00361A06970039A";
        public const string KINECT_B_ID = @"USB\VID_045E&PID_02AE\A00366A21910044A";

        public const int IMG_WIDTH = 640;
        public const int IMG_HEIGHT = 480;

        //Valor RGB para solo ver las esferas
        public const int THRESHOLD = 150;

        public const int MIN_DEPTH = 500;   //mm
        public const int MAX_DEPTH = 1500;  //mm

        //pixeles para buscar depth válido alrededor del centroide
        public const int SEARCH_RADIUS = 15;

        //Peripherical sampling 
        public const int DEPTH_MIN_SAMPLES = 3;

        //Parámetros de detección de blobs
        public const int MIN_BLOB_AREA = 5; //Píxeles mínimos, 0 para aumentar sensibilidad a larga distancia (+1m)
        public const int MAX_BLOB_AREA = 1000; //Píxeles máximos, vamos a siempre tener una distancia prudente
        public const double MIN_CIRCULARITY = 0.3; //Esferas son redondas (~1.0), rechaza líneas/ruido
        public const double MIN_ASPECT = 0.6; //Aspect ratio mínimo (rechaza líneas alargadas, como herramientas)
        public const double MAX_ASPECT = 1.7;

        public const float FOCAL_PX = 585f;   // focal IR efectiva; refinar con el barrido / intrínsecos
        public const float SPHERE_MM = 12.5f;
        public const int Z_SIZE_TOL = 130;   // mm; generoso porque Z_size es grosero

        //KAlman coasting
        public const double COAST_SECONDS = 0.5;
        public const double MAX_VEL = 2000.0; // mm/s

        // Diagnóstico: vuelca la composición de la corona de profundidad por blob.
        // Se activa desde Program.cs cuando el modo es Profiling (no const: se asigna en runtime).
        public static bool VERBOSE_DEPTH = false;
    }
}