declare module 'node:assert/strict' {
  interface StrictAssert {
    equal(actual: unknown, expected: unknown, message?: string): void;
    deepEqual(actual: unknown, expected: unknown, message?: string): void;
    ok(value: unknown, message?: string): asserts value;
  }

  const assert: StrictAssert;
  export default assert;
}

declare module 'node:test' {
  type TestBody = () => void | Promise<void>;
  interface TestOptions {
    todo?: boolean | string;
  }
  interface TestFunction {
    (name: string, body: TestBody): void;
    (name: string, options: TestOptions, body: TestBody): void;
  }

  const test: TestFunction;
  export default test;
}
