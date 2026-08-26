namespace KinectTracker
{//Clase de la mayoría de constantes del sistema, para experimentación fácil.
    public static class Constants
    {
        //IDs de los Kinects
        public const string KINECT_A_ID = @"USB\VID_045E&PID_02AE\A00361A06970039A";
        public const string KINECT_B_ID = @"USB\VID_045E&PID_02AE\A00366A21910044A";

        //Visualizacion y filtrado de imagen
        public const int IMG_WIDTH = 640;
        public const int IMG_HEIGHT = 480;
                
        public const int THRESHOLD = 100;

        public const int MIN_DEPTH = 500;   //mm
        public const int MAX_DEPTH = 1500;  //mm

        //pixeles para buscar depth válido alrededor del centroide
        //public const int SEARCH_RADIUS = 15;

        //Peripherical sampling 
        public const int DEPTH_MIN_SAMPLES = 3;

        //Parámetros de detección de blobs
        public const int MIN_BLOB_AREA = 5; //Píxeles mínimos, 0 para aumentar sensibilidad a larga distancia (+1m)
        public const int MAX_BLOB_AREA = 1000; //Píxeles máximos, vamos a siempre tener una distancia prudente
        public const double MIN_CIRCULARITY = 0.3; //Esferas son redondas (~1.0), rechaza líneas/ruido
        public const double MIN_ASPECT = 0.6; //Aspect ratio mínimo (rechaza líneas alargadas, como herramientas)
        public const double MAX_ASPECT = 1.7;

        //Trackeo, matcheo, filtrado y coasting de bolas de los instrumentos
        public const float MARKER_MEMORY_RADIUS = 20.0f;   //mm; las bolas del marcador no se mueven
        public const int AUDIO_HYSTERESIS = 5;   //frames que hay que aguantar antes de cambiar de estado

        public const float CLUSTER_RADIUS = 130.0f;   //algo mas que los 107.28mm del modelo, margen de ruido
        public const int MIN_NEIGHBORS = 2;

        public const float PREDICTION_RADIUS = 60.0f;   //el salto de punta llega a ~50mm en movimientos rapidos
        public const int MIN_KEPT = 3;                  //por debajo de 3 no hay ni partial: no filtramos

        public const float COLD_TOLERANCE = 8.0f;    //estricto: mas que el match normal
        public const float COLD_MAX_RESIDUAL = 2.5f; //solo trios que cuadran muy bien
        public const float COLD_MAX_POSE_ERR = 6.0f;

        public const double COAST_SECONDS = 0.5;
        public const double MAX_VEL = 2000.0; //mm/s
        public const int MAX_COAST = 6;

        //Caracterización del sistema
        public const int PROFILE_FRAMES = 990;   //~30s a 33fps
        public const long SYNC_TOL_MS = 15;   //medio frame a 30fps

        //Para futuro modo debug? Entonces no debería de ser const.
        public static bool VERBOSE_DEPTH = false;
    }
}