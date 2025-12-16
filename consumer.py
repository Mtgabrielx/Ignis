import pika, threading, time

def make_callback(csv_path: str):
    def callback(ch,method,properties,body):
        string_value = body.decode()

        with open(csv_path,"+a", encoding="utf-8") as file:
            file.write(string_value+'\n')
        
    return callback

class RabbitmqConsumer:
    def __init__(self, callback, queue):
        self.__host = ""
        self.__port = 5672
        self.__username = ""
        self.__password = ""
        self.__queue = queue
        self.__callback = callback
        self.__connection = None
        self.__channel = self.__create_channel()
    
    def __create_channel(self):
        self.connection_parameters = pika.ConnectionParameters(
            host=self.__host,
            port=self.__port,
            heartbeat=60,                 # envia heartbeats a cada 60s
            socket_timeout=300,           # operações de socket podem bloquear até 5 min
            blocked_connection_timeout=600, # aguarda até 10 min se o broker bloquear
            credentials=pika.PlainCredentials(
                username=self.__username,
                password=self.__password
            )
        )

    # def consumer_channel(self):
        self.__connection = pika.BlockingConnection(self.connection_parameters)
        channel = self.__connection.channel()
        channel.queue_declare(
            queue=self.__queue,
            durable=True
        )

        channel.basic_consume(
            queue=self.__queue,
            auto_ack=True,
            on_message_callback=self.__callback
        )

        return channel

    def start(self):
        print(f"[{self.__queue}] começando")
        try:
            self.__channel.start_consuming()
        except Exception as e:
            print(f"[{self.__queue}] saiu do consumo: {e}")
        finally:
            self.__close()
        
    def stop(self):
        try:
            if self.__channel and self.__channel.is_open:
                self.__channel.stop_consuming()
        except Exception:
            pass

    def __close(self):
        try:
            if self.__channel and self.__channel.is_open:
                self.__channel.close()
        except Exception:
            pass
        try:
            if self.__connection and self.__connection.is_open:
                self.__connection.close()
        except Exception:
            pass

if __name__ == "__main__":
    filas = ["temperatura", "umidade", "fumaca", "gas"]  
    consumers = []
    threads = []

    for q in filas:
        cb = make_callback(f"{q}.csv")
        consumer = RabbitmqConsumer(cb,queue=q)
        t = threading.Thread(target=consumer.start, name=f"consumer-{q}", daemon=True)
        consumers.append(consumer)
        threads.append(t)
        t.start()
    print("Consumidores iniciados (um por fila). Ctrl+C para encerrar.")
    
    try:
    # Mantém o main vivo enquanto as threads rodam
        while any(t.is_alive() for t in threads):
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nInterrompido. Encerrando…")
        for c in consumers:
            c.stop()
        for t in threads:
            t.join(timeout=5)
        print("Todos os consumidores finalizados.")

