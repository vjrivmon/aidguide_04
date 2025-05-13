declare module 'roslib' {
    namespace ROSLIB {
      class Ros {
        constructor(options: { url: string });
        on(event: "connection" | "error" | "close", callback: (data?: any) => void): void;
        close(): void;
        isConnected: boolean;
      }

      class Topic {
        constructor(options: {
          ros: Ros;
          name: string;
          messageType: string;
          queue_size?: number;
          throttle_rate?: number;
        });
        subscribe(callback: (message: any) => void): void;
        unsubscribe(): void;
        advertise(): void;
        publish(message: Message): void;
      }

      class Message {
        constructor(values: any);
      }

      class Service {
        constructor(options: {
          ros: Ros;
          name: string;
          serviceType: string;
        });
        callService(request: any, callback: (response: any) => void, failedCallback?: (error: any) => void): void;
      }

      class ActionClient {
        constructor(options: {
          ros: Ros;
          serverName: string;
          actionName: string;
        });
        
        goal(goal: any): Goal;
      }

      class Goal {
        constructor(options: {
          actionClient: ActionClient;
          goalMessage: any;
        });
        
        send(timeout?: number): void;
        on(event: "status" | "feedback" | "result", callback: (data: any) => void): void;
        cancel(): void;
      }
    }
    export default ROSLIB;
  }