declare module 'roslib' {
    namespace ROSLIB {
      class Ros {
        constructor(options: { url: string });
        on(event: "connection" | "error" | "close", callback: (data?: any) => void): void;
        close(): void;
      }
    }
    export default ROSLIB;
  }