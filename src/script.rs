use rhai::{Engine, Scope, AST};
use std::error::Error;

pub struct Script<'a> {
    ast: AST,
    scope: Scope<'a>,
}

impl<'a> Script<'a> {
    pub(crate) fn new(engine: &Engine, source: impl AsRef<str>) -> Result<Self, Box<dyn Error>> {
        let mut scope = Scope::new();
        let ast = engine.compile_with_scope(&scope, source)?;
        engine.run_ast_with_scope(&mut scope, &ast)?;
        Ok(Self { ast, scope })
    }
}
