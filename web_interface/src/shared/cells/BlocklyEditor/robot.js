import Blockly from "blockly";

Blockly.Robot = new Blockly.Generator("Robot");

Blockly.Robot.addReservedWords(
  "Blockly," + // In case JS is evaled in the current window.
    "Robot"
);

/**
 * Order of operation ENUMs.
 * https://developer.mozilla.org/en/Robot/Reference/Operators/Operator_Precedence
 */
Blockly.Robot.ORDER_ATOMIC = 0; // 0 "" ...
Blockly.Robot.ORDER_NEW = 1.1; // new
Blockly.Robot.ORDER_MEMBER = 1.2; // . []
Blockly.Robot.ORDER_FUNCTION_CALL = 2; // ()
Blockly.Robot.ORDER_INCREMENT = 3; // ++
Blockly.Robot.ORDER_DECREMENT = 3; // --
Blockly.Robot.ORDER_BITWISE_NOT = 4.1; // ~
Blockly.Robot.ORDER_UNARY_PLUS = 4.2; // +
Blockly.Robot.ORDER_UNARY_NEGATION = 4.3; // -
Blockly.Robot.ORDER_LOGICAL_NOT = 4.4; // !
Blockly.Robot.ORDER_TYPEOF = 4.5; // typeof
Blockly.Robot.ORDER_VOID = 4.6; // void
Blockly.Robot.ORDER_DELETE = 4.7; // delete
Blockly.Robot.ORDER_AWAIT = 4.8; // await
Blockly.Robot.ORDER_EXPONENTIATION = 5.0; // **
Blockly.Robot.ORDER_MULTIPLICATION = 5.1; // *
Blockly.Robot.ORDER_DIVISION = 5.2; // /
Blockly.Robot.ORDER_MODULUS = 5.3; // %
Blockly.Robot.ORDER_SUBTRACTION = 6.1; // -
Blockly.Robot.ORDER_ADDITION = 6.2; // +
Blockly.Robot.ORDER_BITWISE_SHIFT = 7; // << >> >>>
Blockly.Robot.ORDER_RELATIONAL = 8; // < <= > >=
Blockly.Robot.ORDER_IN = 8; // in
Blockly.Robot.ORDER_INSTANCEOF = 8; // instanceof
Blockly.Robot.ORDER_EQUALITY = 9; // == != === !==
Blockly.Robot.ORDER_BITWISE_AND = 10; // &
Blockly.Robot.ORDER_BITWISE_XOR = 11; // ^
Blockly.Robot.ORDER_BITWISE_OR = 12; // |
Blockly.Robot.ORDER_LOGICAL_AND = 13; // &&
Blockly.Robot.ORDER_LOGICAL_OR = 14; // ||
Blockly.Robot.ORDER_CONDITIONAL = 15; // ?:
Blockly.Robot.ORDER_ASSIGNMENT = 16; // = += -= **= *= /= %= <<= >>= ...
Blockly.Robot.ORDER_YIELD = 17; // yield
Blockly.Robot.ORDER_COMMA = 18; // ,
Blockly.Robot.ORDER_NONE = 99; // (...)

/**
 * List of outer-inner pairings that do NOT require parentheses.
 * @type {!Array.<!Array.<number>>}
 */
Blockly.Robot.ORDER_OVERRIDES = [
  // (foo()).bar -> foo().bar
  // (foo())[0] -> foo()[0]
  [Blockly.Robot.ORDER_FUNCTION_CALL, Blockly.Robot.ORDER_MEMBER],
  // (foo())() -> foo()()
  [Blockly.Robot.ORDER_FUNCTION_CALL, Blockly.Robot.ORDER_FUNCTION_CALL],
  // (foo.bar).baz -> foo.bar.baz
  // (foo.bar)[0] -> foo.bar[0]
  // (foo[0]).bar -> foo[0].bar
  // (foo[0])[1] -> foo[0][1]
  [Blockly.Robot.ORDER_MEMBER, Blockly.Robot.ORDER_MEMBER],
  // (foo.bar)() -> foo.bar()
  // (foo[0])() -> foo[0]()
  [Blockly.Robot.ORDER_MEMBER, Blockly.Robot.ORDER_FUNCTION_CALL],

  // !(!foo) -> !!foo
  [Blockly.Robot.ORDER_LOGICAL_NOT, Blockly.Robot.ORDER_LOGICAL_NOT],
  // a * (b * c) -> a * b * c
  [Blockly.Robot.ORDER_MULTIPLICATION, Blockly.Robot.ORDER_MULTIPLICATION],
  // a + (b + c) -> a + b + c
  [Blockly.Robot.ORDER_ADDITION, Blockly.Robot.ORDER_ADDITION],
  // a && (b && c) -> a && b && c
  [Blockly.Robot.ORDER_LOGICAL_AND, Blockly.Robot.ORDER_LOGICAL_AND],
  // a || (b || c) -> a || b || c
  [Blockly.Robot.ORDER_LOGICAL_OR, Blockly.Robot.ORDER_LOGICAL_OR],
];

// /**
// * Initialise the database of letiable names.
// * @param {!Blockly.Workspace} workspace Workspace to generate code from.
// */
// Blockly.Robot.init = function(workspace) {
// // Create a dictionary of definitions to be printed before the code.
// Blockly.Robot.definitions_ = Object.create(null);
// // Create a dictionary mapping desired function names in definitions_
// // to actual function names (to avoid collisions with user functions).
// Blockly.Robot.functionNames_ = Object.create(null);

// if (!Blockly.Robot.letiableDB_) {
// Blockly.Robot.letiableDB_ =
// new Blockly.Names(Blockly.Robot.RESERVED_WORDS_);
// } else {
// Blockly.Robot.letiableDB_.reset();
// }

// Blockly.Robot.letiableDB_.setVariableMap(workspace.getVariableMap());

// let deflets = [];
// // Add developer letiables (not created or named by the user).
// let devVarList = Blockly.Variables.allDeveloperVariables(workspace);
// for (let i = 0; i < devVarList.length; i++) {
// deflets.push(Blockly.Robot.letiableDB_.getName(devVarList[i],
// Blockly.Names.DEVELOPER_VARIABLE_TYPE));
// }

// // Add user letiables, but only ones that are being used.
// let letiables = Blockly.Variables.allUsedVarModels(workspace);
// for (let i = 0; i < letiables.length; i++) {
// deflets.push(Blockly.Robot.letiableDB_.getName(letiables[i].getId(),
// Blockly.Variables.NAME_TYPE));
// }

// // Declare all of the letiables.
// if (deflets.length) {
// Blockly.Robot.definitions_['letiables'] =
// 'let ' + deflets.join(', ') + ';';
// }
// };

// /**
// * Prepend the generated code with the letiable definitions.
// * @param {string} code Generated code.
// * @return {string} Completed code.
// */
// Blockly.Robot.finish = function(code) {
// // Convert the definitions dictionary into a list.
// let definitions = [];
// for (let name in Blockly.Robot.definitions_) {
// definitions.push(Blockly.Robot.definitions_[name]);
// }
// // Clean up temporary data.
// delete Blockly.Robot.definitions_;
// delete Blockly.Robot.functionNames_;
// Blockly.Robot.letiableDB_.reset();
// return definitions.join('\n\n') + '\n\n\n' + code;
// };

/**
 * Naked values are top-level blocks with outputs that aren't plugged into
 * anything.  A trailing semicolon is needed to make this legal.
 * @param {string} line Line of generated code.
 * @return {string} Legal line of code.
 */
// Blockly.Robot.scrubNakedValue = function(line) {
// return line + ';\n';
// };

/**
 * Encode a string as a properly escaped Robot string, complete with
 * quotes.
 * @param {string} string Text to encode.
 * @return {string} Robot string.
 * @private
 */
// Blockly.Robot.quote_ = function(string) {
// // Can't use goog.string.quote since Google's style guide recommends
// // JS string literals use single quotes.
// string = string.replace(/\\/g, '\\\\')
// .replace(/\n/g, '\\\n')
// .replace(/'/g, '\\\'');
// return '\'' + string + '\'';
// };

/**
 * Common tasks for generating Robot from blocks.
 * Handles comments for the specified block and any connected value blocks.
 * Calls any statements following this block.
 * @param {!Blockly.Block} block The current block.
 * @param {string} code The Robot code created for this block.
 * @param {boolean=} opt_thisOnly True to generate code for only this statement.
 * @return {string} Robot code with comments and subsequent blocks added.
 * @private
 */
/*
Blockly.Robot.blockToCode = function(a, b) {
  //console.log(a)
  if (!a) return "";
  if (!a.isEnabled()) return b ? "": this.blockToCode(a.getNextBlock());
  var c = this[a.type];
  //console.log("c的类型为：",c)
  if ("function" != typeof c) throw Error('Language "' + this.name_ + '" does not know how to generate  code for block type "' + a.type + '".');
  c = c.call(a, a);
  //console.log("c.call的值为：",c)
  if (Array.isArray(c)) {
      if (!a.outputConnection) throw TypeError("Expecting string from statement block: " + a.type);
      return [this.scrub_(a, c[0], b), c[1]]
  }
  if ("string" == typeof c) return this.STATEMENT_PREFIX && !a.suppressPrefixSuffix && (c = this.injectId(this.STATEMENT_PREFIX, a) + c),
  this.STATEMENT_SUFFIX && !a.suppressPrefixSuffix && (c += this.injectId(this.STATEMENT_SUFFIX, a)),
  this.scrub_(a, c, b);
  if (null === c) return "";
  throw SyntaxError("Invalid code generated: " + c);
};
*/
Blockly.Robot.scrub_ = function (block, code, optThisOnly) {
  const nextBlock = block.nextConnection && block.nextConnection.targetBlock();
  const nextCode = optThisOnly ? "" : Blockly.Robot.blockToCode(nextBlock);
  return code + nextCode;
};

/**
 * Gets a property and adjusts the value while taking into account indexing.
 * @param {!Blockly.Block} block The block.
 * @param {string} atId The property ID of the element to get.
 * @param {number=} opt_delta Value to add.
 * @param {boolean=} opt_negate Whether to negate the value.
 * @param {number=} opt_order The highest order acting on this value.
 * @return {string|number}
 */
// Blockly.Robot.getAdjusted = function(block, atId, opt_delta, opt_negate,
// opt_order) {
// let delta = opt_delta || 0;
// let order = opt_order || Blockly.Robot.ORDER_NONE;
// if (block.workspace.options.oneBasedIndex) {
// delta--;
// }
// let defaultAtIndex = block.workspace.options.oneBasedIndex ? '1' : '0';
// let at = ''
// if (delta > 0) {
// at = Blockly.Robot.valueToCode(block, atId,
// Blockly.Robot.ORDER_ADDITION) || defaultAtIndex;
// } else if (delta < 0) {
// at = Blockly.Robot.valueToCode(block, atId,
// Blockly.Robot.ORDER_SUBTRACTION) || defaultAtIndex;
// } else if (opt_negate) {
// at = Blockly.Robot.valueToCode(block, atId,
// Blockly.Robot.ORDER_UNARY_NEGATION) || defaultAtIndex;
// } else {
// at = Blockly.Robot.valueToCode(block, atId, order) ||
// defaultAtIndex;
// }

// if (Blockly.isNumber(at)) {
// // If the index is a naked number, adjust it right now.
// at = parseFloat(at) + delta;
// if (opt_negate) {
// at = -at;
// }
// } else {
// // If the index is dynamic, adjust it in code.
// let innerOrder = null
// if (delta > 0) {
// at = at + ' + ' + delta;
// innerOrder = Blockly.Robot.ORDER_ADDITION;
// } else if (delta < 0) {
// at = at + ' - ' + -delta;
// innerOrder = Blockly.Robot.ORDER_SUBTRACTION;
// }
// if (opt_negate) {
// if (delta) {
// at = '-(' + at + ')';
// } else {
// at = '-' + at;
// }
// innerOrder = Blockly.Robot.ORDER_UNARY_NEGATION;
// }
// innerOrder = Math.floor(innerOrder);
// order = Math.floor(order);
// if (innerOrder && order >= innerOrder) {
// at = '(' + at + ')';
// }
// }
// return at;
// };
