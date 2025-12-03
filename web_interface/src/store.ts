// import { createStore, applyMiddleware, compose } from 'redux'
import { configureStore, EnhancedStore } from "@reduxjs/toolkit";
import createSagaMiddleware, { END, Task, Saga } from "redux-saga";
import rootReducer from "./state/reducers";

interface ExtendedStore extends EnhancedStore {
  runSaga: <S extends Saga<any[]>>(
    saga: S,
    ...args: Parameters<S>
  ) => Task<any>;
  close: () => void;
}

export default function configureAppStore(): ExtendedStore {
  const sagaMiddleware = createSagaMiddleware();
  const store = configureStore({
    reducer: rootReducer,
    middleware: [sagaMiddleware],
  });

  if (process.env.NODE_ENV !== "production" && Boolean(module.hot)) {
    module.hot?.accept("./state/reducers", () => {
      // eslint-disable-next-line @typescript-eslint/no-var-requires
      const nextRootReducer = require("./state/reducers").default;
      store.replaceReducer(nextRootReducer);
    });
  }
  return {
    ...configureStore({
      reducer: rootReducer,
      middleware: [sagaMiddleware],
    }),
    runSaga: sagaMiddleware.run,
    close: () => store.dispatch(END),
  };
}

// export default function configureStore (initialState): Store<any, any, [any]> {
//   const sagaMiddleware = createSagaMiddleware()
//   const store = createStore(
//     rootReducer,
//     initialState,
//     composeEnhancers(applyMiddleware(sagaMiddleware))
//   )

//   if (module.hot) {
//     module.hot.accept('./state/reducers', () => {
//       const nextRootReducer = require('./state/reducers').default
//       store.replaceReducer(nextRootReducer)
//     })
//   }
//   store.runSaga = sagaMiddleware.run
//   store.close = () => store.dispatch(END)
//   return store
// }
