import ROSLIB from 'roslib';

// Clase para manejar la comunicación con ROS2
class ROSService {
  private ros: ROSLIB.Ros | null = null;
  private connectionListeners: ((connected: boolean) => void)[] = [];
  private readonly rosbridgeAddress = "ws://localhost:9090";
  
  // Singleton
  private static instance: ROSService;
  
  private constructor() {
    this.initConnection();
  }
  
  // Obtener la instancia única del servicio
  public static getInstance(): ROSService {
    if (!ROSService.instance) {
      ROSService.instance = new ROSService();
    }
    return ROSService.instance;
  }
  
  // Iniciar la conexión con ROS
  private initConnection() {
    this.ros = new ROSLIB.Ros({
      url: this.rosbridgeAddress
    });

    this.ros.on('connection', () => {
      console.log('Conectado a ROSBridge');
      this.notifyConnectionListeners(true);
    });

    this.ros.on('error', (error) => {
      console.error('Error de conexión a ROSBridge:', error);
      this.notifyConnectionListeners(false);
    });

    this.ros.on('close', () => {
      console.log('Conexión a ROSBridge cerrada');
      this.notifyConnectionListeners(false);
      
      // Intentar reconectar automáticamente después de un tiempo
      setTimeout(() => {
        this.initConnection();
      }, 5000);
    });
  }
  
  // Añadir listener para el estado de la conexión
  public addConnectionListener(listener: (connected: boolean) => void) {
    this.connectionListeners.push(listener);
    // Notificar inmediatamente del estado actual
    if (this.ros) {
      listener(this.isConnected());
    }
  }
  
  // Eliminar listener
  public removeConnectionListener(listener: (connected: boolean) => void) {
    const index = this.connectionListeners.indexOf(listener);
    if (index !== -1) {
      this.connectionListeners.splice(index, 1);
    }
  }
  
  // Notificar a todos los listeners
  private notifyConnectionListeners(connected: boolean) {
    this.connectionListeners.forEach(listener => listener(connected));
  }
  
  // Verificar si está conectado
  public isConnected(): boolean {
    return this.ros !== null && this.ros.isConnected;
  }
  
  // Iniciar la ruta de navegación siguiendo waypoints
  public startWaypointFollowing(): boolean {
    if (!this.isConnected()) {
      console.error('No hay conexión con ROS');
      return false;
    }
    
    try {
      const startWaypointPub = new ROSLIB.Topic({
        ros: this.ros!,
        name: '/start_waypoint_following',
        messageType: 'std_msgs/Bool'
      });
      
      const message = new ROSLIB.Message({
        data: true
      });
      
      startWaypointPub.publish(message);
      console.log('Comando de inicio de navegación enviado');
      return true;
    } catch (error) {
      console.error('Error al publicar mensaje de inicio de navegación:', error);
      return false;
    }
  }
  
  // Detener la navegación
  public stopWaypointFollowing(): boolean {
    if (!this.isConnected()) {
      console.error('No hay conexión con ROS');
      return false;
    }
    
    try {
      const stopWaypointPub = new ROSLIB.Topic({
        ros: this.ros!,
        name: '/stop_waypoint_following',
        messageType: 'std_msgs/Bool'
      });
      
      const message = new ROSLIB.Message({
        data: true
      });
      
      stopWaypointPub.publish(message);
      console.log('Comando de detención de navegación enviado');
      return true;
    } catch (error) {
      console.error('Error al publicar mensaje de detención de navegación:', error);
      return false;
    }
  }
  
  // Obtener el estado actual de la navegación
  public getNavigationStatus(callback: (status: string, waypoint: number) => void) {
    if (!this.isConnected()) {
      console.error('No hay conexión con ROS');
      return;
    }
    
    const statusListener = new ROSLIB.Topic({
      ros: this.ros!,
      name: '/navigation_status',
      messageType: 'std_msgs/String'
    });
    
    statusListener.subscribe((message: any) => {
      callback(message.data, 0); // Podríamos parsear el mensaje para obtener el waypoint actual
    });
    
    return statusListener;
  }
}

export default ROSService; 