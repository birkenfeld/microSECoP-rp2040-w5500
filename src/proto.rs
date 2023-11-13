// -----------------------------------------------------------------------------
// Rust SECoP playground
//
// This program is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation; either version 2 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Module authors:
//   Georg Brandl <g.brandl@fz-juelich.de>
//   Enrico Faulhaber <enrico.faulhaber@frm2.tum.de>
//
// -----------------------------------------------------------------------------
//
//! SECoP data type / data info definitions.

use core::marker::PhantomData;
use core::fmt;
use core::fmt::Write as _;
use w5500_dhcp::hl::io::Write;


pub type Result<'a, T> = core::result::Result<T, Error<'a>>;

#[derive(Debug)]
pub enum ErrorKind {
    // Internal
    Config,
    Programming,
    Parsing,
    // API defined
    Protocol,
    NoSuchModule,
    NoSuchParameter,
    NoSuchCommand,
    CommandFailed,
    CommandRunning,
    ReadOnly,
    BadValue,
    CommunicationFailed,
    Timeout,       // ATM also C.F.
    HardwareError, // ATM also C.F.
    IsBusy,
    IsError,
    Disabled,
}

#[derive(Debug)]
pub struct Error<'a> {
    kind: ErrorKind,
    message: &'static str,
    #[allow(unused)]
    original: Option<&'a str>,
}

impl<'a> Error<'a> {
    pub fn new(kind: ErrorKind, message: &'static str, original: &'a str) -> Self {
        Self { kind, message, original: Some(original) }
    }

    pub fn into_msg(self, req: &'a str, spec: &'a str) -> Msg<'a> {
        Msg::ErrMsg {
            request: req,
            specifier: spec,
            class: self.wire(),
            message: self.message,
        }
    }

    fn wire(&self) -> &'static str {
        use self::ErrorKind::*;
        match self.kind {
            Config | Programming | Parsing => "InternalError",
            Protocol => "ProtocolError",
            NoSuchModule => "NoSuchModule",
            NoSuchParameter => "NoSuchParameter",
            NoSuchCommand => "NoSuchCommand",
            CommandFailed => "CommandFailed",
            CommandRunning => "CommandRunning",
            ReadOnly => "ReadOnly",
            BadValue => "BadValue",
            CommunicationFailed => "CommunicationFailed",
            Timeout => "CommunicationFailed",
            HardwareError => "CommunicationFailed",
            IsBusy => "IsBusy",
            IsError => "IsError",
            Disabled => "Disabled",
        }
    }

    // Quick construction.

    pub fn bad_value(message: &'static str) -> Self {
        Self { kind: ErrorKind::BadValue, message, original: None }
    }

    pub fn config(message: &'static str) -> Self {
        Self { kind: ErrorKind::Config, message, original: None }
    }

    pub fn protocol(message: &'static str) -> Self {
        Self { kind: ErrorKind::Protocol, message, original: None }
    }

    pub fn no_module() -> Self {
        Self { kind: ErrorKind::NoSuchModule, message: "", original: None }
    }

    pub fn no_param() -> Self {
        Self { kind: ErrorKind::NoSuchParameter, message: "", original: None }
    }

    pub fn no_command() -> Self {
        Self { kind: ErrorKind::NoSuchCommand, message: "", original: None }
    }

    pub fn comm_failed(message: &'static str) -> Self {
        Self { kind: ErrorKind::CommunicationFailed, message, original: None }
    }
}


pub const IDENT_REPLY: &str = "ISSE&SINE2020,SECoP,V2019-09-16,v1.0\n";

/// Enum that represents any message that can be sent over the network in the
/// protocol, and some others that are only used internally.
#[derive(Debug, Clone)]
pub enum Msg<'a> {
    /// identify request
    Idn,
    /// identify reply
    IdnReply,
    /// description request
    Describe,
    /// description reply
    Describing { id: &'a str, structure: &'a str },
    /// event enable request
    Activate { module: &'a str },
    /// event enable reply
    Active { module: &'a str },
    /// event disable request
    Deactivate { module: &'a str },
    /// event disable reply
    Inactive { module: &'a str },
    /// command execution request
    Do { module: &'a str, command: &'a str, arg: &'a str },
    /// command result
    Done { module: &'a str, command: &'a str, data: &'a str },
    /// change request
    Change { module: &'a str, param: &'a str, value: &'a str },
    /// change result
    Changed { module: &'a str, param: &'a str, data: &'a str },
    /// read request
    Read { module: &'a str, param: &'a str },
    /// heartbeat request
    Ping { token: &'a str },
    /// heartbeat reply
    Pong { token: &'a str, data: &'a str },
    /// error reply
    ErrMsg { request: &'a str, specifier: &'a str, class: &'a str, message: &'a str },
    /// update event
    Update { module: &'a str, param: &'a str, data: &'a str },

    // /// not a protocol message, but a collection of initial updates
    // InitUpdates { module: &'a str, updates: Vec<Msg> },
    // /// not a protocol message, indicates the connection is done
    // Quit,
}

/// An incoming message that carries around the originating line from the
/// client.  We need this line for the error message if something goes wrong.
#[derive(Clone)]
pub struct IncomingMsg<'a>(pub &'a str, pub Msg<'a>);

use self::Msg::*;

mod wire {
    pub const IDN: &str = "*IDN?";
    pub const DESCRIBE: &str = "describe";
    pub const DESCRIBING: &str = "describing";
    pub const ACTIVATE: &str = "activate";
    pub const ACTIVE: &str = "active";
    pub const DEACTIVATE: &str = "deactivate";
    pub const INACTIVE: &str = "inactive";
    pub const PING: &str = "ping";
    pub const PONG: &str = "pong";
    pub const ERROR: &str = "error";
    pub const DO: &str = "do";
    pub const DONE: &str = "done";
    pub const CHANGE: &str = "change";
    pub const CHANGED: &str = "changed";
    pub const READ: &str = "read";
    pub const UPDATE: &str = "update";
}

impl Msg<'_> {
    /// Parse a string slice containing a message.
    ///
    /// This matches a regular expression, and then creates a `Msg` if successful.
    pub fn parse(msg: &[u8]) -> core::result::Result<IncomingMsg, Msg> {
        match core::str::from_utf8(msg) {
            Err(_) => Err(Error::protocol("invalid UTF8").into_msg("", "")),
            Ok(msg) => match Self::parse_inner(&msg) {
                Ok(v) => Ok(IncomingMsg(msg, v)),
                Err(e) => Err(e.into_msg("", "")),
            }
        }
    }

    fn parse_inner(msg: &str) -> Result<Msg> {
        let mut parts = msg.trim().splitn(3, " ");

        let action = parts.next().ok_or(Error::protocol("missing action"))?;
        let specifier = parts.next().unwrap_or_default();
        let data = parts.next().unwrap_or_default();
        let mut spec_split = specifier.splitn(2, ':').map(Into::into);
        let module = spec_split.next().expect("cannot be absent");
        let mut param = || spec_split.next().ok_or(Error::protocol("missing parameter"));

        let parsed = match action {
            wire::READ =>       Read { module, param: param()? },
            wire::CHANGE =>     Change { module, param: param()?, value: data },
            wire::DO =>         Do { module, command: param()?, arg: data },
            wire::DESCRIBE =>   Describe,
            wire::ACTIVATE =>   Activate { module },
            wire::DEACTIVATE => Deactivate { module },
            wire::PING =>       Ping { token: specifier.into() },
            wire::IDN =>        Idn,
            wire::UPDATE =>     Update { module, param: param()?, data },
            _ => return Err(Error::protocol("message type not supported"))
        };

        Ok(parsed)
    }

    fn fmt<E, T: Write<E>>(&self, mut w: FmtWrap<'_, E, T>) -> fmt::Result {
        match self {
            Update { module, param, data } =>
                write!(w, "{} {}:{} {}\n", wire::UPDATE, module, param, data),
            Changed { module, param, data } =>
                write!(w, "{} {}:{} {}\n", wire::CHANGED, module, param, data),
            Done { module, command, data } =>
                write!(w, "{} {}:{} {}\n", wire::DONE, module, command, data),
            Describing { id, structure } =>
                write!(w, "{} {} {}\n", wire::DESCRIBING, id, structure),
            Active { module } =>
                if module.is_empty() { w.write_str(wire::ACTIVE) }
                else { write!(w, "{} {}\n", wire::ACTIVE, module) },
            Inactive { module } =>
                if module.is_empty() { w.write_str(wire::INACTIVE) }
                else { write!(w, "{} {}\n", wire::INACTIVE, module) },
            Pong { token, data } =>
                write!(w, "{} {} {}\n", wire::PONG, token, data),
            Idn => w.write_str(wire::IDN),
            IdnReply => w.write_str(IDENT_REPLY),
            Read { module, param } =>
                write!(w, "{} {}:{}\n", wire::READ, module, param),
            Change { module, param, value } =>
                write!(w, "{} {}:{} {}\n", wire::CHANGE, module, param, value),
            Do { module, command, arg } =>
                write!(w, "{} {}:{} {}\n", wire::DO, module, command, arg),
            Describe => w.write_str(wire::DESCRIBE),
            Activate { module } =>
                if module.is_empty() { w.write_str(wire::ACTIVATE) }
                else { write!(w, "{} {}\n", wire::ACTIVATE, module) },
            Deactivate { module } =>
                if module.is_empty() { w.write_str(wire::DEACTIVATE) }
                else { write!(w, "{} {}\n", wire::DEACTIVATE, module) },
            Ping { token } =>
                if token.is_empty() { w.write_str(wire::PING) }
                else { write!(w, "{} {}\n", wire::PING, token) },
            ErrMsg { request, specifier, class, message } =>
                write!(w, "{}_{} {} [\"{}\",\"{}\",{{}}]\n",
                       wire::ERROR, request, specifier, class, message),
        }
    }
}

struct FmtWrap<'a, E, T: Write<E>>(&'a mut T, PhantomData<E>);

impl<'a, E, T: Write<E>> core::fmt::Write for FmtWrap<'a, E, T> {
    fn write_str(&mut self, msg: &str) -> core::result::Result<(), core::fmt::Error> {
        self.0.write(msg.as_bytes()).map_err(|_| core::fmt::Error).map(|_| ())
    }
}

impl<'a> IncomingMsg<'a> {
    pub fn bare(msg: Msg<'a>) -> Self {
        IncomingMsg("", msg)
    }
}

impl fmt::Display for IncomingMsg<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl fmt::Display for Error<'_> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}: {}", self.wire(), self.message)
    }
}


pub struct SecNode {
}

impl SecNode {
    pub fn new() -> Self {
        Self {}
    }

    pub fn process<E>(&mut self, input: &[u8], writer: &mut impl Write<E>) {
        let writer = FmtWrap(writer, PhantomData);
        let reply = match Msg::parse(input) {
            Ok(input_msg) => match input_msg.1 {
                Msg::Idn => Msg::IdnReply,
                Msg::Ping { token } => Msg::Pong { token, data: "{}" },
                Msg::Change { module, param, value } => return self.change(module, param, value, writer),
                Msg::Read { module, param } => return self.read(module, param, writer),
                Msg::Do { module, command, arg } => return self.do_(module, command, arg, writer),
                Msg::Describe => todo!(),
                _ => Error::protocol("not implemented").into_msg("", ""),
            },
            Err(e) => e,
        };
        let _ = reply.fmt(writer);
    }

    fn read<E, T: Write<E>>(&mut self, module: &str, param: &str, writer: FmtWrap<'_, E, T>) {
        let _ = Msg::Update { module, param, data: "0" }.fmt(writer);
    }

    fn change<E, T: Write<E>>(&mut self, module: &str, param: &str, value: &str, writer: FmtWrap<'_, E, T>) {
        let _ = Msg::Changed { module, param, data: value }.fmt(writer);
    }

    fn do_<E, T: Write<E>>(&mut self, module: &str, command: &str, arg: &str, writer: FmtWrap<'_, E, T>) {
        let _ = Msg::Done { module, command, data: arg }.fmt(writer);
    }
}
