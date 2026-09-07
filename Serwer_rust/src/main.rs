// Host-side control client for the OSOYOO 2WD AGV running on an ESP32.
//
// Connects over Wi-Fi (TCP) to the firmware in OSOYOO/src/main.cpp and provides
// a manual command console for bringing up motors and line sensors.
// Standard library only, no external crates.
//
// Usage:
//   cargo run                        # auto-discovers the robot via UDP broadcast
//   cargo run -- <ip>[:port]         # connect to a known address
//
// NOTE: motor A is wired to K1 and motor B to K3 (K2/K4 unused), so each motor
// is driven by a single half of the H-bridge and spins in one direction only.
// Negative speeds simply stop the motors.

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, UdpSocket};
use std::time::{Duration, Instant};

const DEFAULT_PORT: u16 = 8080;
const DISCOVERY_PORT: u16 = 8266;

/// The firmware broadcasts "OSOYOO <ip> <port>" once per second over UDP, so the
/// robot's address does not have to be known up front.
fn discover(timeout: Duration) -> std::io::Result<String> {
    let socket = UdpSocket::bind(("0.0.0.0", DISCOVERY_PORT))?;
    socket.set_read_timeout(Some(Duration::from_millis(500)))?;
    let deadline = Instant::now() + timeout;
    let mut buf = [0u8; 128];

    println!("Searching for the robot (UDP :{DISCOVERY_PORT}) ...");
    while Instant::now() < deadline {
        match socket.recv_from(&mut buf) {
            Ok((n, src)) => {
                let msg = String::from_utf8_lossy(&buf[..n]);
                let parts: Vec<&str> = msg.split_whitespace().collect();
                if parts.first() == Some(&"OSOYOO") {
                    let ip = parts.get(1).copied().unwrap_or("");
                    let port = parts.get(2).copied().unwrap_or("8080");
                    let ip = if ip.is_empty() {
                        src.ip().to_string()
                    } else {
                        ip.to_string()
                    };
                    println!("Robot found at {ip}:{port}");
                    return Ok(format!("{ip}:{port}"));
                }
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue
            }
            Err(e) => return Err(e),
        }
    }
    Err(std::io::Error::new(
        std::io::ErrorKind::TimedOut,
        "robot not found - check that the ESP32 is on the same Wi-Fi network and \
         that UDP port 8266 is not blocked by a firewall (or pass the address \
         explicitly: cargo run -- 192.168.0.x)",
    ))
}

struct Robot {
    reader: BufReader<TcpStream>,
    writer: TcpStream,
}

impl Robot {
    fn connect(addr: &str) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(Duration::from_secs(3)))?;
        let mut robot = Robot {
            reader: BufReader::new(stream.try_clone()?),
            writer: stream,
        };
        let hello = robot.read_line()?; // greeting from the firmware
        println!("<- {hello}");
        Ok(robot)
    }

    fn read_line(&mut self) -> std::io::Result<String> {
        let mut line = String::new();
        if self.reader.read_line(&mut line)? == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "connection closed by the ESP32",
            ));
        }
        Ok(line.trim_end().to_string())
    }

    fn send(&mut self, cmd: &str) -> std::io::Result<String> {
        write!(self.writer, "{cmd}\n")?;
        self.writer.flush()?;
        let resp = self.read_line()?;
        println!("<- {resp}");
        Ok(resp)
    }
}

fn console(robot: &mut Robot) -> std::io::Result<()> {
    println!("\nManual control console");
    println!("  M <left> <right>   set motor speeds (0..255; refresh within 2 s");
    println!("                     or the firmware watchdog stops the motors)");
    println!("  S                  stop");
    println!("  SENS               read line sensors (left / centre / right)");
    println!("  INFO               IP, RSSI, uptime");
    println!("  PING               link check");
    println!("  q                  quit\n");

    let stdin = std::io::stdin();
    loop {
        print!("robot> ");
        std::io::stdout().flush()?;
        let mut line = String::new();
        if stdin.lock().read_line(&mut line)? == 0 {
            break;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if line == "q" || line == "quit" {
            break;
        }
        robot.send(line)?;
    }
    robot.send("S")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let target = args.get(1).map(String::as_str).unwrap_or("auto");

    let addr = if target == "auto" {
        match discover(Duration::from_secs(20)) {
            Ok(a) => a,
            Err(e) => {
                eprintln!("Error: {e}");
                std::process::exit(1);
            }
        }
    } else if target.contains(':') {
        target.to_string()
    } else {
        format!("{target}:{DEFAULT_PORT}")
    };

    println!("Connecting to {addr} ...");
    let mut robot = match Robot::connect(&addr) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Connection failed: {e}");
            std::process::exit(1);
        }
    };

    if let Err(e) = console(&mut robot) {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
    println!("Disconnected.");
}
